/*
 * STM32F767ZI_i2c_driver.c
 *
 *  Created on: Sep 14, 2023
 *      Author: rahim
 */
#include "STM32F767ZI_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

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

uint32_t RCC_GetPLLOutputClock()
{
	// TODO: Write function to determine PLL output clock
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0)
	{
		// HSI clock source
		SystemClk = 16000000;
	}
	else if (clksrc == 1)
	{
		// HSE clock source
		SystemClk = 8000000;

	}
	else if (clksrc == 2)
	{
		// PLL is not used in this course
		// TODO: Write function to determine PLL output clock
		SystemClk = RCC_GetPLLOutputClock();
	}

	// Determine AHB prescaler from register
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// Determine APB1 prescaler from register
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;
	return pclk1;
}

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
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

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
 *
 * @return				- none
 *
 * @Note 				- none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr)
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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

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
