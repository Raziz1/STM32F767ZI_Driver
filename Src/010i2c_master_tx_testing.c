/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Sep 14, 2023
 *      Author: rahim
 */

#include <stdio.h>
#include <string.h>
#include "STM32F767ZI.h"

I2C_Handle_t I2C1Handle;
#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68
//Test data transmission
uint8_t some_data[] = "HELLOWORLD\n";

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

// PB8 -> I2C1_SCL
// PB9 -> I2C1_SDA

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; // only needed if slave mode (@see protocol reference for reserved addresses)
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM100K;
    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIObtn;
	//GPIO button
	GPIObtn.pGPIOx = GPIOC;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIObtn);
}

int main(void)
{
	//Initialize user button
	GPIO_ButtonInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

    while (1)
    {
        // wait for button press
        while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        // send some data to the slave
        I2C_MasterSendData(&I2C1Handle, some_data, strlen((char *)some_data), SLAVE_ADDR);
    };}
