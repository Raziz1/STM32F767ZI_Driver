/*
 * STM32F767ZI_i2c_driver.c
 *
 *  Created on: Sep 14, 2023
 *      Author: rahim
 */
#include "STM32F767ZI_i2c_driver.h"

/*********************************************************************************************
 *   Hard-coded I2C_TIMINGR timing settings depending on the I2C clock
 *      See tables in ref manual, section 33.4.9
 *     | SM_10kHz | SM_100kHz | FM_400kHz | FM+_1000kHz |
 *  source: https://github.com/MayaPosch/Nodate/blob/master/arch/stm32/cpp/core/src/i2c.cpp
 * Note: STM32CubeMX calculates and provides the I2C_TIMINGR content in the I2C Config. window.
 * *******************************************************************************************/
uint32_t i2c_timings_4[4]  = {0x004091F3, 0x00400D10, 0x00100002, 0x00000001};
uint32_t i2c_timings_8[4]  = {0x1042C3C7, 0x10420F13, 0x00310309, 0x00100306};
uint32_t i2c_timings_16[4] = {0x3042C3C7, 0x303D5B, 0x10320309, 0x00200204}; // SM_100k timing value from STM32CubeMX, for avoiding glitch (it works)
uint32_t i2c_timings_48[4] = {0xB042C3C7, 0xB0420F13, 0x50330309, 0x50100103};
uint32_t i2c_timings_54[4] = {0xD0417BFF, 0x40D32A31, 0x10A60D20, 0x00900916};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= (1 << I2C_CR2_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	// Place the slave address in the appropriate 7:1 register (master mode)
	pI2Cx->CR2 |= (SlaveAddr << 1);
	// Configure transfer direction as read (master mode)
	pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	// Place the slave address in the appropriate 7:1 register (master mode)
	pI2Cx->CR2 |= (SlaveAddr << 1);
	// Configure transfer direction as write (master mode)
	pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);
}


/****************************************************************
 * @fn					- I2C_PeriClockControl
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Enable or disable
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_DI();
		}
	}
}

/****************************************************************
 * @fn					- I2C_PeripheralControl
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Enable or disable the I2C peripheral
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}


/****************************************************************
 * @fn					- I2C_Init
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    // Enable the clock for the I2Cx peripheral
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
    // Initialize as slave mode
    pI2CHandle->Mode = I2C_MODE_SLAVE;

    // Program the device own address (Using 7-bit slave address)
    uint32_t tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->OAR1 = tempreg;
    // Own address 1 is a 7-bit address
    pI2CHandle->pI2Cx->OAR1 &= ~(1 << I2C_OAR1_OA1MODE);
    // Own Address 1 enabled
    pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OAR1_OA1EN);

    // fill I2C_TIMINGR register using hard-coded timings values
    uint8_t mode = 0;
    mode = pI2CHandle->I2C_Config.I2C_SCLSpeed; // mode: SM10k, SM100k, FM or FMPLUS
    if (RCC_GetPCLK1Value() == 8000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_8[mode];
    }else if (RCC_GetPCLK1Value() == 16000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_16[mode];
    }else if (RCC_GetPCLK1Value() == 48000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_48[mode];
    }else if (RCC_GetPCLK1Value() == 54000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_54[mode];
    }
}

/****************************************************************
 * @fn					- I2C_DeInit
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
	else if (pI2Cx == I2C4)
	{
		I2C4_REG_RESET();
	}
}

/****************************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Base address of the Tx buffer
 * @param[in]			- Length of the Tx buffer
 * @param[in]			- Slave address
 * @param[in]			- Repeated Start
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);
	//Clear automatic end mode bit to software end mode (TC flag is set when NBYTES data are transferred, stretching SCL low)
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
	//Set the number of bytes to be transmitted
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);

	// Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Transmit data until Len of data becomes 0
	while (Len > 0)
	{
		// Wait until the transmit data register is empty
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->TXDR = *pTxbuffer;
		pTxbuffer ++;
		Len --;
	}

	// Wait until the transfer complete register is set
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC));

	// Generate a stop bit
    if (Sr == I2C_DISABLE_SR)
    {
        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

	// Clear STOPF flag
    pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
    // Clear I2C_CR2 registers
    pI2CHandle->pI2Cx->CR2 = 0x0;
}

/****************************************************************
 * @fn					- I2C_MasterReceiveData
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Base address of the Rx buffer
 * @param[in]			- Length of the Rx buffer
 * @param[in]			- Slave address
 * @param[in]			- Repeated Start
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//Clear automatic end mode bit to software end mode (TC flag is set when NBYTES data are transferred, stretching SCL low)
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
	//Set the number of bytes to be received
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);

	// Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Receive data until Len of data becomes 0
	while (Len > 0)
	{
		// Wait until the receive data register is empty
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		*pRxbuffer = pI2CHandle->pI2Cx->RXDR;
		pRxbuffer ++;
		Len --;
	}
	// Wait until the transfer complete register is set
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC));

	// Generate a stop bit
    if (Sr == I2C_DISABLE_SR)
    {
        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

	// Clear STOPF flag
    pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
    // Clear I2C_CR2 registers
    pI2CHandle->pI2Cx->CR2 = 0x0;
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/****************************************************************
 * @fn					- I2C_IRQInterruptConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- I2C IRQ configure priority function
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn					- I2C_IRQPriorityConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- I2C IRQ configure priority function
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * @fn					- I2C_MasterSendDataIT
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Base address of the Tx buffer
 * @param[in]			- Length of the Tx buffer
 * @param[in]			- Slave address
 * @param[in]			- Repeated Start
 *
 * @return				- uint8_t
 *
 * @Note 				- None blocking interrupt based I2C transmission
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    // Check transmission and receive state
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Mode = I2C_MODE_MASTER;
        pI2CHandle->Sr = Sr;

        // Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		//Clear automatic end mode bit to software end mode (TC flag is set when NBYTES data are transferred, stretching SCL low)
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
		//Set the number of bytes to be transmitted
		pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->TxLen << I2C_CR2_NBYTES);

        //Implement code to Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //enable TXIE, TCIE, STOPIE interrupt control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);

        //enable ERRIE interrupt control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);
	}

	return busystate;
}

/****************************************************************
 * @fn					- I2C_MasterReceiveDataIT
 *
 * @brief				-
 *
 * @param[in]			- Base address of the I2C peripheral
 * @param[in]			- Base address of the Tx buffer
 * @param[in]			- Length of the Tx buffer
 * @param[in]			- Slave address
 * @param[in]			- Repeated Start
 *
 * @return				- uint8_t
 *
 * @Note 				- None blocking interrupt based I2C receive
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;
    // Check transmission and receive state
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Mode = I2C_MODE_MASTER;
		pI2CHandle->Sr = Sr;

		// Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		//Clear automatic end mode bit to software end mode (TC flag is set when NBYTES data are transferred, stretching SCL low)
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
		//Set the number of bytes to be transmitted
		pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->TxLen << I2C_CR2_NBYTES);

        //Implement code to Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//enable RXIE, STOPIE, TCIE Control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE);
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);

		//enable ERRIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);

	}

	return busystate;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    //disable TXIE, STOPIE, TCIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    //disable RXIE, STOPIE, TCIE, NACKIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
}

/****************************************************************
 * @fn					- I2C_EV_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the I2C handle
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;
    // Handle for interrupt generated by ADDR event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ADDRIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_ADDR);
    if (temp1 && temp2)
    {
        // clear flag by setting the ADDRCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_ADDRCF);
        /* Enable Address Acknowledge */
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_NACK);
    }

    // Handle for interrupt generated by received NACK event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_NACKIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_NACKF);
    if (temp1 && temp2)
    {
        // clear flag by setting the NACKCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_NACKCF);

        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_NACK);
    }

    // Handle for interrupt generated by TC event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TCIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TC);
    if (temp1 && temp2)
    {
            //1. generate the STOP condition
            if(pI2CHandle->Sr == I2C_DISABLE_SR)
            {
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            // 2. reset all the member elements of the handle structure.
            I2C_CloseSendData(pI2CHandle);

            pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register

    }

    // Handle for interrupt generated by STOPF event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_STOPIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_STOPF);
    if (temp1 && temp2)
    {
        // STOPF flag is set
        // - set by hardware when a Stop condition is detected.
        // - cleared by software by setting the STOPCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);

        //Notify the application that STOP is detected
        //I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // Handle for interrupt generated by TXIS event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TXIS);
    if (temp1 && temp2)
    {
        // TXIS flag is set

        //check for device mode
        if (pI2CHandle->Mode == I2C_MODE_MASTER)
        {
            // master: We have to do the data transmission
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                if (pI2CHandle->TxLen > 0)
                {
                    // 1. load the data in TXDR
                    pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
                    // 2. decrement TxLen
                    pI2CHandle->TxLen--;
                    // 3. increment buffer address
                    pI2CHandle->pTxBuffer++;
                }
            }
        }
        else
        {
            //slave
            // check that slave is in transmitter mode (DIR bit set in ISR register)
            if (pI2CHandle->pI2Cx->ISR & (1<<I2C_ISR_DIR))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }
    // Handle for interrupt generated by RXNE event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_RXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_RXNE);
    if (temp1 && temp2)
    {
        // RXNE flag is set
        //check for device mode
        if (pI2CHandle->Mode == I2C_MODE_MASTER)
        {
            // master mode
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                if (pI2CHandle->RxLen > 0)
                {
                    // 1. read RXDR data into buffer
                    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
                    // 2. decrement RxLen
                    pI2CHandle->RxLen--;
                    // 3. increment buffer address
                    pI2CHandle->pRxBuffer++;
                }

                if(pI2CHandle->RxLen == 0)
                {
                    //1. generate the stop condition
                    if(pI2CHandle->Sr == I2C_DISABLE_SR){
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    //2 . Close the I2C rx
                    I2C_CloseReceiveData(pI2CHandle);

                    //3. Notify the application
                    I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);

                    pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
                }

            }
        }
        else
        {
            // slave mode
            // check that slave is in receiver mode (DIR bit reset in ISR register)
            if (!(pI2CHandle->pI2Cx->ISR & (1<<I2C_ISR_DIR)))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

/****************************************************************
 * @fn					- I2C_ER_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the I2C handle
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1,temp2;

    //Know the status of ERRIE control bit in CR1
    temp2 = (pI2CHandle->pI2Cx->CR1) & ( 1 << I2C_CR1_ERRIE);


    /***********************Check for Bus error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_BERR);
    if(temp1  && temp2 )
    {
        //This is Bus error

        //clear the bus error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_BERRCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_ARLO);
    if(temp1  && temp2)
    {
        //This is arbitration lost error

        //clear the arbitration lost error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_ARLOCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_OVR);
    if(temp1  && temp2)
    {
        //This is Overrun/underrun error

        //clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_OVRCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
    }

    /***********************Check for Time out error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_TIMEOUT);
    if(temp1  && temp2)
    {
        //This is Time out error

        //clear the Time out error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_TIMEOUTCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
    }
}

/****************************************************************
 * @fn					- I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the I2C handle
 * @param[in]			- Enable/Disable
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        //enable TXIE, RXIE, STOPIE, ADDRIE, NACKIE Control Bits
		pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ADDRIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_NACKIE);
    }
    else
    {
        //disable TXIE, RXIE, STOPIE, ADDRIE, NACKIE Control Bits
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE);
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE);
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_ADDRIE);
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_NACKIE);
    }
}


/****************************************************************
 * @fn					- I2C_SlaveSendData
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the I2C handle
 * @param[in]			- Data to send byte by byte
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->TXDR = data;
}

/****************************************************************
 * @fn					- I2C_SlaveReceiveData
 *
 * @brief				-
 *
 * @param[in]			- Pointer to the I2C handle
 *
 * @return				- none
 *
 * @Note 				- none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t)pI2Cx->RXDR;
}
