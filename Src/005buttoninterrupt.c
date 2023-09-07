/*
 * 005buttoninterrupt.c
 *
 *  Created on: Sep 6, 2023
 *      Author: rahim
 */
#include <stdint.h>
#include <string.h>
#include "STM32F767ZI.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOled, GPIObtn;
	memset(&GPIOled, 0, sizeof(GPIOled));
	memset(&GPIObtn, 0, sizeof(GPIObtn));

	//GPIO led
	GPIOled.pGPIOx = GPIOB;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO button
	GPIObtn.pGPIOx = GPIOC;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOled);
	GPIO_Init(&GPIObtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while (1);
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);  //Clear the pending event from the EXTI line
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
}



