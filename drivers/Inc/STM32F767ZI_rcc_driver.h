/*
 * STM32F767ZI_rcc_driver.h
 *
 *  Created on: Sep 15, 2023
 *      Author: rahim
 */

#ifndef INC_STM32F767ZI_RCC_DRIVER_H_
#define INC_STM32F767ZI_RCC_DRIVER_H_

#include "STM32F767ZI.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F767ZI_RCC_DRIVER_H_ */
