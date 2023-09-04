/*
 * 001ledtoggle.c
 *
 *  Created on: Sep 3, 2023
 *      Author: rahim
 */
#include <stdint.h>
#include "STM32F767ZI.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(void)
{
	for (uint32_t i = 0; i < 700000; i++);
}

int main(void)
{
	GPIO_Handle_t Gpioled;
	Gpioled.pGPIOx = GPIOB;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&Gpioled);

	while (1)
	{
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		delay();
	}
	return 0;
}

