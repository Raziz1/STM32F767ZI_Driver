/*
 * STM32F767ZI.h
 *
 *  Created on: Aug 14, 2023
 *      Author: rahim
 */

#ifndef INC_STM32F767ZI_H_
#define INC_STM32F767ZI_H_

#include <stdint.h>

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20020000U /* 368 kB */
#define SRAM2_BASEADDR 0x2007C000U /* 16 kB */
#define ROM_BASEADDDR  0x00100000U
#define SRAM		   SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDDR     0x40000000U
#define APB1PERIPH_BASEADDDR PERIPH_BASEADDDR
#define APB2PERIPH_BASEADDDR 0x40010000U
#define AHB1PERIPH_BASEADDDR 0x40020000U
#define AHB2PERIPH_BASEADDDR 0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x0000U)
#define GPIOB_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x0400U)
#define GPIOC_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x0800U)
#define GPIOD_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x0C00U)
#define GPIOE_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x1000U)
#define GPIOF_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x1400U)
#define GPIOG_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x1800U)
#define GPIOH_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x1C00U)
#define GPIOI_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x2000U)
#define GPIOJ_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x2400U)
#define GPIOK_BASEADDR     (AHB1PERIPH_BASEADDDR + 0x2800U)
#define RCC_BASEADDR       (AHB1PERIPH_BASEADDDR + 0x3800U)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR     (APB1PERIPH_BASEADDDR + 0x5400U)
#define I2C2_BASEADDR     (APB1PERIPH_BASEADDDR + 0x5800U)
#define I2C3_BASEADDR     (APB1PERIPH_BASEADDDR + 0x5C00U)
#define I2C4_BASEADDR     (APB1PERIPH_BASEADDDR + 0x6000U)

#define SPI2_BASEADDR     (APB1PERIPH_BASEADDDR + 0x3800U)
#define SPI3_BASEADDR     (APB1PERIPH_BASEADDDR + 0x3C00U)

#define USART2_BASEADDR   (APB1PERIPH_BASEADDDR + 0x4400U)
#define USART3_BASEADDR   (APB1PERIPH_BASEADDDR + 0x4800U)
#define UART4_BASEADDR    (APB1PERIPH_BASEADDDR + 0x4C00U)
#define UART5_BASEADDR    (APB1PERIPH_BASEADDDR + 0x5000U)
#define UART7_BASEADDR    (APB1PERIPH_BASEADDDR + 0x7800U)
#define UART8_BASEADDR    (APB1PERIPH_BASEADDDR + 0x7C00U)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR   (APB2PERIPH_BASEADDDR + 0x3C00U)

#define SPI1_BASEADDR   (APB2PERIPH_BASEADDDR + 0x3000U)
#define SPI4_BASEADDR   (APB2PERIPH_BASEADDDR + 0x3400U)
#define SPI5_BASEADDR   (APB2PERIPH_BASEADDDR + 0x5000U)
#define SPI6_BASEADDR   (APB2PERIPH_BASEADDDR + 0x5400U)

#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDDR + 0x3800U)

#define USART1_BASEADDR (APB2PERIPH_BASEADDDR + 0x1000U)
#define USART6_BASEADDR (APB2PERIPH_BASEADDDR + 0x1400U)


/******************** Peripheral register definition structures ********************/

typedef struct
{
	volatile uint32_t MODER;           /*GPIO port mode register*/
	volatile uint32_t OTYPER;		   /*GPIO port output type register*/
	volatile uint32_t OSPEEDR;         /*GPIO port output speed register*/
	volatile uint32_t PUPDR;           /*GPIO port pull-up/pull-down register*/
	volatile uint32_t IDR;             /*GPIO port input data register*/
	volatile uint32_t ODR;             /*GPIO port output data register*/
	volatile uint32_t BSRR;            /*GPIO port bit set/reset register*/
	volatile uint32_t LCKR;            /*GPIO port configuration lock register*/
	volatile uint32_t AFR[2];          /*GPIO alternate low/high register*/
}GPIO_RegDef_t;


/******************** RCC register definition structures ********************/

typedef struct
{
	volatile uint32_t CR;        /*TODO: FUNCTION*/
	volatile uint32_t PLLCFGR;	 /*TODO: FUNCTION*/
	volatile uint32_t CFGR;      /*TODO: FUNCTION*/
	volatile uint32_t CIR;       /*TODO: FUNCTION*/
	volatile uint32_t AHB1RSTR;  /*TODO: FUNCTION*/
	volatile uint32_t AHB2RSTR;  /*TODO: FUNCTION*/
	volatile uint32_t AHB3RSTR;  /*TODO: FUNCTION*/
	uint32_t RESERVED0;          /*TODO: FUNCTION*/
	volatile uint32_t APB1RSTR;  /*TODO: FUNCTION*/
	volatile uint32_t APB2RSTR;  /*TODO: FUNCTION*/
	uint32_t RESERVED1[2];       /*TODO: FUNCTION*/
	volatile uint32_t AHB1ENR;   /*TODO: FUNCTION*/
	volatile uint32_t AHB2ENR;   /*TODO: FUNCTION*/
	volatile uint32_t AHB3ENR;   /*TODO: FUNCTION*/
	uint32_t RESERVED2;          /*TODO: FUNCTION*/
	volatile uint32_t APB1ENR;   /*TODO: FUNCTION*/
	volatile uint32_t APB2ENR;   /*TODO: FUNCTION*/
	uint32_t RESERVED3[2];       /*TODO: FUNCTION*/
	volatile uint32_t AHB1LPENR; /*TODO: FUNCTION*/
	volatile uint32_t AHB2LPENR; /*TODO: FUNCTION*/
	volatile uint32_t AHB3LPENR; /*TODO: FUNCTION*/
	uint32_t RESERVED4;          /*TODO: FUNCTION*/
	volatile uint32_t APB1LPENR; /*TODO: FUNCTION*/
	volatile uint32_t APB2LPENR; /*TODO: FUNCTION*/
	uint32_t RESERVED5[2];       /*TODO: FUNCTION*/
	volatile uint32_t BDCR;      /*TODO: FUNCTION*/
	volatile uint32_t CSR;       /*TODO: FUNCTION*/
	uint32_t RESERVED6[2];       /*TODO: FUNCTION*/
	volatile uint32_t SSCGR;     /*TODO: FUNCTION*/
	volatile uint32_t PLLI2SCFGR;/*TODO: FUNCTION*/
	volatile uint32_t PLLSAICFGR;/*TODO: FUNCTION*/
	volatile uint32_t DCKCFGR1;  /*TODO: FUNCTION*/
	volatile uint32_t DCKCFGR2;  /*TODO: FUNCTION*/
}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
 */
#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH   ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI   ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ   ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK   ((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0));
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1 << 1));
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1 << 2));
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1 << 3));
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1 << 4));
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1 << 5));
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1 << 6));
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1 << 7));
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1 << 8));
#define GPIOJ_PCLK_EN()  (RCC->AHB1ENR |= (1 << 9));
#define GPIOK_PCLK_EN()  (RCC->AHB1ENR |= (1 << 10));

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 0));
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 1));
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 2));
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 3));
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 4));
#define GPIOF_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 5));
#define GPIOG_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 6));
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 7));
#define GPIOI_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 8));
#define GPIOJ_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 9));
#define GPIOK_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 10));

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()  (RCC->APB2ENR |= (1 << 12));
#define SPI2_PCLK_EN()  (RCC->APB1ENR |= (1 << 14));
#define SPI3_PCLK_EN()  (RCC->APB1ENR |= (1 << 15));
#define SPI4_PCLK_EN()  (RCC->APB2ENR |= (1 << 13));
#define SPI5_PCLK_EN()  (RCC->APB2ENR |= (1 << 20));
#define SPI6_PCLK_EN()  (RCC->APB2ENR |= (1 << 21));

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()  (RCC->APB2ENR &| ~(1 << 12));
#define SPI2_PCLK_DI()  (RCC->APB1ENR &| ~(1 << 14));
#define SPI3_PCLK_DI()  (RCC->APB1ENR &| ~(1 << 15));
#define SPI4_PCLK_DI()  (RCC->APB2ENR &| ~(1 << 13));
#define SPI5_PCLK_DI()  (RCC->APB2ENR &| ~(1 << 20));
#define SPI6_PCLK_DI()  (RCC->APB2ENR &| ~(1 << 21));

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1 << 21));
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1 << 22));
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1 << 23));
#define I2C4_PCLK_EN()  (RCC->APB1ENR |= (1 << 24));

/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 21));
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 22));
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 23));
#define I2C4_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 24));

/*
 * Clock enable macros for UARTx peripherals
 */
#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1 << 4));
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1 << 17));
#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1 << 18));
#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1 << 19));
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1 << 20));
#define USART6_PCLK_EN()  (RCC->APB2ENR |= (1 << 5));
#define UART7_PCLK_EN()  (RCC->APB1ENR |= (1 << 30));
#define UART8_PCLK_EN()  (RCC->APB1ENR |= (1 << 31));

/*
 * Clock disable macros for UARTx peripherals
 */
#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 4));
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 17));
#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 18));
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 19));
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 20));
#define USART6_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 5));
#define UART7_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 30));
#define UART8_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 31));

/*
 * Clock enable macros for SYSCONFIG peripherals
 */
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1 << 14));

/*
 * Clock disable macros for SYSCONFIG peripherals
 */
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 14));

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR |= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR |= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR |= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR |= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR |= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR |= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR |= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR |= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR |= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR |= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR |= ~(1 << 10)); } while(0)

// Some generic macros
#define ENABLE        1
#define DISABLE       0
#define SET           (ENABLE)
#define RESET         (DISABLE)
#define GPIO_PIN_SET  (SET)
#define GPIO_PIN_RESET(RESET)

#include "STM32F767ZI_gpio_driver.h"


#endif /* INC_STM32F767ZI_H_ */
