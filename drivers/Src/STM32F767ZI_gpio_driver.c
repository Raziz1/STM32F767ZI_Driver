/*
 * STM32F767ZI_gpio_driver.c
 *
 *  Created on: Aug 22, 2023
 *      Author: rahim
 */
#include "STM32F767ZI_gpio_driver.h"
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
	}
}

/****************************************************************
 * @fn					- GPIO_Init
 *
 * @brief				-
 *
 * @param[in]			- Base address of the GPIO peripheral
 * @param[in]
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting
	}
	else /* Interrupt mode */
	{

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2 * 4);

		//3.Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp = 0;

	//2. Configure the speed of the GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure the alt function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
}

/****************************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO pin number
 * @param[in]
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if (pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/****************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO pin number
 * @param[in]			- GPIO pin number
 * @param[in]
 *
 * @return				- 0 or 1
 *
 * @Note 				- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

/****************************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO pin number
 * @param[in]
 * @param[in]
 *
 * @return				- Returns value of all GPIO pins for a port
 *
 * @Note 				- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/****************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO pin number
 * @param[in]			- GPIO pin number
 * @param[in]			- value to output (0 or 1)
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bitfield corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/****************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO pin number
 * @param[in]			- value to output (0 or 1)
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				-
 *
 * @param[in]			- Pointer to GPIO port
 * @param[in]			- GPIO pin number
 * @param[in]
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}

/****************************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				-
 *
 * @param[in]			- IRQNumber
 * @param[in]			- Enable or Disable
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/****************************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- PinNumber
 *
 * @return				- none
 *
 * @Note 				- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear EXTI pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

