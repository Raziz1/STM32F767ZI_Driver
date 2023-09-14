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


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
