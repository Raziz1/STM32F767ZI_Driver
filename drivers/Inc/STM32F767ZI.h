/*
 * STM32F767ZI.h
 *
 *  Created on: Aug 14, 2023
 *      Author: rahim
 */

#ifndef INC_STM32F767ZI_H_
#define INC_STM32F767ZI_H_

#include <stdint.h>
#include <stddef.h>
/*****************************************START: Processor Specific Details **************************************/
/*
 * ARM Cortex M7 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0	((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1	((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2	((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3	((volatile uint32_t*)0xE000E10C)
/*
 * ARM Cortex M7 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0	((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1	((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2	((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3	((volatile uint32_t*)0xE000E18C)

/*
 * ARM Cortex M7 Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR ((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4
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
	volatile uint32_t MODER;           /*GPIO port mode register		 		OFFSET: 0x00*/
	volatile uint32_t OTYPER;		   /*GPIO port output type register 		OFFSET: 0x04*/
	volatile uint32_t OSPEEDR;         /*GPIO port output speed register 		OFFSET: 0x08*/
	volatile uint32_t PUPDR;           /*GPIO port pull-up/pull-down register	OFFSET: 0x0C*/
	volatile uint32_t IDR;             /*GPIO port input data register			OFFSET: 0x10*/
	volatile uint32_t ODR;             /*GPIO port output data register			OFFSET: 0x14*/
	volatile uint32_t BSRR;            /*GPIO port bit set/reset register		OFFSET: 0x18*/
	volatile uint32_t LCKR;            /*GPIO port configuration lock register	OFFSET: 0x1C*/
	volatile uint32_t AFR[2];          /*GPIO alternate low/high register		OFFSET: 0x20-0x24*/
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

/******************** EXTI register definition structures ********************/

typedef struct
{
	volatile uint32_t IMR;           /*TODO: FUNCTION*/
	volatile uint32_t EMR;		     /*TODO: FUNCTION*/
	volatile uint32_t RTSR;		     /*TODO: FUNCTION*/
	volatile uint32_t FTSR;		     /*TODO: FUNCTION*/
	volatile uint32_t SWIER;		 /*TODO: FUNCTION*/
	volatile uint32_t PR;		     /*TODO: FUNCTION*/
}EXTI_RegDef_t;

/******************** SYSCFG register definition structures ********************/

typedef struct
{
	volatile uint32_t MEMRMP;        /*TODO: FUNCTION*/
	volatile uint32_t PMC;		     /*TODO: FUNCTION*/
	volatile uint32_t EXTICR[4];	 /*TODO: FUNCTION*/
	uint32_t RESERVED1[2];		     /*TODO: FUNCTION*/
	volatile uint32_t CMPCR;		 /*TODO: FUNCTION*/
	uint32_t RESERVED[2];		     /*TODO: FUNCTION*/
	volatile uint32_t CFGR;		     /*TODO: FUNCTION*/
}SYSCFG_RegDef_t;

/******************** Peripheral register definition for SPI ********************/

typedef struct
{
	volatile uint32_t CR1;         /**/
	volatile uint32_t CR2;		   /**/
	volatile uint32_t SR;          /**/
	volatile uint32_t DR;          /**/
	volatile uint32_t CRCPR;       /**/
	volatile uint32_t RXCRCR;      /**/
	volatile uint32_t TXCRCR;      /**/
	volatile uint32_t I2SCFGR;     /**/
	volatile uint32_t I2SPR;       /**/
}SPI_RegDef_t;

/******************** Peripheral register definition for I2C ********************/

typedef struct
{
	volatile uint32_t CR1;         /**/
	volatile uint32_t CR2;		   /**/
	volatile uint32_t OAR1;        /**/
	volatile uint32_t OAR2;        /**/
	volatile uint32_t TIMINGR;          /**/
	volatile uint32_t TIMEOUTR;         /**/
	volatile uint32_t ISR;         /**/
	volatile uint32_t ICR;         /**/
	volatile uint32_t PECR;       /**/
	volatile uint32_t RXDR;        /**/
	volatile uint32_t TXDR;        /**/
}I2C_RegDef_t;

/******************** Peripheral register definition for USART ********************/

typedef struct
{
    volatile uint32_t CR1;    // USART Control Register 1
    volatile uint32_t CR2;    // USART Control Register 2
    volatile uint32_t CR3;    // USART Control Register 3
    volatile uint32_t BRR;    // USART Baud rate register
    volatile uint32_t GTPR;   // USART Guard time and prescaler register
    volatile uint32_t RTOR;   // USART Receiver timeout register
    volatile uint32_t RQR;    // USART Request register
    volatile uint32_t ISR;    // USART Interrupt and status register
    volatile uint32_t ICR;    // USART Interrupt flag clear register
    volatile uint32_t RDR;    // USART Receive data register
    volatile uint32_t TDR;    // USART Transmit data register
}USART_RegDef_t;

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

#define SPI1   ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2   ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3   ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4   ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5   ((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6   ((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1   ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2   ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3   ((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4   ((I2C_RegDef_t*)I2C4_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define USART1  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4   ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5   ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  ((USART_RegDef_t*)USART6_BASEADDR)
#define UART7   ((USART_RegDef_t*)UART7_BASEADDR)
#define UART8   ((USART_RegDef_t*)UART8_BASEADDR)
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
#define SPI1_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 12));
#define SPI2_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 14));
#define SPI3_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 15));
#define SPI4_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 13));
#define SPI5_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 20));
#define SPI6_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 21));

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
#define UART4_PCLK_EN()   (RCC->APB1ENR |= (1 << 19));
#define UART5_PCLK_EN()   (RCC->APB1ENR |= (1 << 20));
#define USART6_PCLK_EN()  (RCC->APB2ENR |= (1 << 5));
#define UART7_PCLK_EN()   (RCC->APB1ENR |= (1 << 30));
#define UART8_PCLK_EN()   (RCC->APB1ENR |= (1 << 31));

/*
 * Clock disable macros for UARTx peripherals
 */
#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 4));
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 17));
#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 18));
#define UART4_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 19));
#define UART5_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 20));
#define USART6_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 5));
#define UART7_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 30));
#define UART8_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 31));

/*
 * Clock enable macros for SYSCONFIG peripherals
 */
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1 << 14));

/*
 * Clock disable macros for SYSCONFIG peripherals
 */
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 14));

/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)     ((x==GPIOA)?0:\
									  (x==GPIOB)?1:\
                                      (x==GPIOC)?2:\
                                      (x==GPIOD)?3:\
                                      (x==GPIOE)?4:\
                                      (x==GPIOF)?5:\
                                      (x==GPIOG)?6:\
                                      (x==GPIOH)?7:\
                                      (x==GPIOI)?8:\
                                      (x==GPIOJ)?9:\
                                      (x==GPIOK)?10:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F767ZI MCU
 * NOTE: update these macros with valid values according to your MCU
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4         84
#define IRQ_NO_SPI5         85
#define IRQ_NO_SPI6         86

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73
#define IRQ_NO_I2C4_EV      95
#define IRQ_NO_I2C4_ER      96

/*
 * Macros for all priority levels
 */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

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

/*
 * Macros to reset all registers of SPIx peripherals
 */
#define SPI1_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR |= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR |= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR |= ~(1 << 15)); } while(0)
#define SPI4_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR |= ~(1 << 13)); } while(0)
#define SPI5_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR |= ~(1 << 20)); } while(0)
#define SPI6_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR |= ~(1 << 21)); } while(0)

/*
 * Macros to reset all registers of I2Cx peripherals
 */
#define I2C1_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR |= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR |= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR |= ~(1 << 23)); } while(0)
#define I2C4_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 24)); (RCC->APB1RSTR |= ~(1 << 24)); } while(0)

#define USART1_REG_RESET()  do{ (RCC->APB2ENR |= (1 << 4));  (RCC->APB2ENR |= ~(1 << 4)); } while(0)
#define USART2_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 17)); (RCC->APB1ENR |= ~(1 << 17)); } while(0)
#define USART3_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 18)); (RCC->APB1ENR |= ~(1 << 18)); } while(0)
#define UART4_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 19)); (RCC->APB1ENR |= ~(1 << 19)); } while(0)
#define UART5_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 20)); (RCC->APB1ENR |= ~(1 << 20)); } while(0)
#define USART6_REG_RESET()  do{ (RCC->APB2ENR |= (1 << 5));  (RCC->APB2ENR |= ~(1 << 5)); } while(0)
#define UART7_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 30)); (RCC->APB1ENR |= ~(1 << 30)); } while(0)
#define UART8_REG_RESET()  do{ (RCC->APB1ENR |= (1 << 31)); (RCC->APB1ENR |= ~(1 << 31)); } while(0)


// Some generic macros
#define ENABLE        	1
#define DISABLE       	0
#define SET           	(ENABLE)
#define RESET         	(DISABLE)
#define GPIO_PIN_SET  	(SET)
#define GPIO_PIN_RESET	(RESET)
#define FLAG_RESET 	  	(RESET)
#define FLAG_SET 		(SET)

/****************************************************************************
 * Bit position definitions of SPI peripheral
 ****************************************************************************/
/*
 * Bit position definitions of SPI_CR1
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 	    11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE 	15

/*
 * Bit position definitions of SPI_CR2
 */
#define SPI_CR2_RXDMAEN 		0
#define SPI_CR2_TXDMAEN 		1
#define SPI_CR2_SSOE 		    2
#define SPI_CR2_FRF 		    4
#define SPI_CR2_ERRIE 		    5
#define SPI_CR2_RXNEIE 		    6
#define SPI_CR2_TXEIE 		    7

/*
 * Bit position definitions of SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/****************************************************************************
 * Bit position definitions of I2C peripheral
 ****************************************************************************/
/*
 * Bit position definitions of I2C_CR1
 */
#define I2C_CR1_PE          0
#define I2C_CR1_TXIE        1
#define I2C_CR1_RXIE        2
#define I2C_CR1_ADDRIE      3
#define I2C_CR1_NACKIE      4
#define I2C_CR1_STOPIE      5
#define I2C_CR1_TCIE        6
#define I2C_CR1_ERRIE       7
#define I2C_CR1_DNF         8
#define I2C_CR1_ANFOFF      12
#define I2C_CR1_TXDMAEN     14
#define I2C_CR1_RXDMAEN     15
#define I2C_CR1_SBC         16
#define I2C_CR1_NOSTRETCH   17
#define I2C_CR1_GCEN        19
#define I2C_CR1_SMBHEN      20
#define I2C_CR1_SMBDEN      21
#define I2C_CR1_ALERTEN     22
#define I2C_CR1_PECEN       23

/*
 * Bit position definitions of I2C_CR2
 */
#define I2C_CR2_SADD        0
#define I2C_CR2_RD_WRN      10
#define I2C_CR2_ADD10       11
#define I2C_CR2_HEAD10R     12
#define I2C_CR2_START       13
#define I2C_CR2_STOP        14
#define I2C_CR2_NACK        15
#define I2C_CR2_NBYTES      16
#define I2C_CR2_RELOAD      24
#define I2C_CR2_AUTOEND     25
#define I2C_CR2_PECBYTE     26

/*
 * Bit position definitions of I2C_OAR1
 */
#define I2C_OAR1_OA1_10BIT  0
#define I2C_OAR1_OA1_7BIT   1
#define I2C_OAR1_OA1MODE    10
#define I2C_OAR1_OA1EN      15

/*
 * Bit position definitions of I2C_TIMINGR
 */
#define I2C_TIMINGR_SCLL    0
#define I2C_TIMINGR_SCLH    8
#define I2C_TIMINGR_SDADEL  16
#define I2C_TIMINGR_SCLDEL  20
#define I2C_TIMINGR_PRESC   28

/*
 * Bit position definitions of I2C_ISR
 */
#define I2C_ISR_TXE         0
#define I2C_ISR_TXIS        1
#define I2C_ISR_RXNE        2
#define I2C_ISR_ADDR        3
#define I2C_ISR_NACKF       4
#define I2C_ISR_STOPF       5
#define I2C_ISR_TC          6
#define I2C_ISR_TCR         7
#define I2C_ISR_BERR        8
#define I2C_ISR_ARLO        9
#define I2C_ISR_OVR         10
#define I2C_ISR_PECERR      11
#define I2C_ISR_TIMEOUT     12
#define I2C_ISR_ALERT       13
#define I2C_ISR_BUSY        15
#define I2C_ISR_DIR         16
#define I2C_ISR_ADDCODE     17


/*
 * Bit position definitions of I2C_ICR
 */
#define I2C_ICR_ADDRCF      3
#define I2C_ICR_NACKCF      4
#define I2C_ICR_STOPCF      5
#define I2C_ICR_BERRCF      8
#define I2C_ICR_ARLOCF      9
#define I2C_ICR_OVRCF       10
#define I2C_ICR_PECCF       11
#define I2C_ICR_TIMEOUTCF   12
#define I2C_ICR_ALERTCF     13

/***********************************************************************************************
 *                      Bit position definitions of USART peripheral
 ***********************************************************************************************/
#define USART_CR1_UE        0
#define USART_CR1_UESM      1
#define USART_CR1_RE        2
#define USART_CR1_TE        3
#define USART_CR1_IDLEIE    4
#define USART_CR1_RXNEIE    5
#define USART_CR1_TCIE      6
#define USART_CR1_TXEIE     7
#define USART_CR1_PEIE      8
#define USART_CR1_PS        9
#define USART_CR1_PCE       10
#define USART_CR1_WAKE      11
#define USART_CR1_M0        12
#define USART_CR1_MME       13
#define USART_CR1_CMIE      14
#define USART_CR1_OVER8     15
#define USART_CR1_DEDT      16
#define USART_CR1_DEAT      21
#define USART_CR1_RTOIE     26
#define USART_CR1_EOBIE     27
#define USART_CR1_M1        28

#define USART_CR2_ADDM7     4
#define USART_CR2_LBDL      5
#define USART_CR2_LBDIE     6
#define USART_CR2_LBCL      8
#define USART_CR2_CPHA      9
#define USART_CR2_CPOL      10
#define USART_CR2_CLKEN     11
#define USART_CR2_STOP      12
#define USART_CR2_LINEN     14
#define USART_CR2_SWAP      15
#define USART_CR2_RXINV     16
#define USART_CR2_TXINV     17
#define USART_CR2_DATAINV   18
#define USART_CR2_MSBFIRST  19
#define USART_CR2_ABREN     20
#define USART_CR2_ABRMOD    21
#define USART_CR2_RTOEN     23
#define USART_CR2_ADD_3_0   24
#define USART_CR2_ADD_7_4   28

#define USART_CR3_EIE       0
#define USART_CR3_IREN      1
#define USART_CR3_IRLP      2
#define USART_CR3_HDSEL     3
#define USART_CR3_NACK      4
#define USART_CR3_SCEN      5
#define USART_CR3_DMAR      6
#define USART_CR3_DMAT      7
#define USART_CR3_RTSE      8
#define USART_CR3_CTSE      9
#define USART_CR3_CTSIE     10
#define USART_CR3_ONEBIT    11
#define USART_CR3_OVRDIS    12
#define USART_CR3_DDRE      13
#define USART_CR3_DEM       14
#define USART_CR3_DEP       15
#define USART_CR3_SCARCNT0  17
#define USART_CR3_SCARCNT1  18
#define USART_CR3_SCARCNT2  19
#define USART_CR3_WUS0      20
#define USART_CR3_WUS1      21
#define USART_CR3_WUFIE     22
#define USART_CR3_UCESM     23
#define USART_CR3_TCBGTIE   24

#define USART_RQR_ABRRQ     0
#define USART_RQR_SBKRQ     1
#define USART_RQR_MMRQ      2
#define USART_RQR_RXFRQ     3
#define USART_RQR_TXFRQ     4

#define USART_ISR_PE        0
#define USART_ISR_FE        1
#define USART_ISR_NF        2
#define USART_ISR_ORE       3
#define USART_ISR_IDLE      4
#define USART_ISR_RXNE      5
#define USART_ISR_TC        6
#define USART_ISR_TXE       7
#define USART_ISR_LBDF      8
#define USART_ISR_CTSIF     9
#define USART_ISR_CTS       10
#define USART_ISR_RTOF      11
#define USART_ISR_EOBF      12
#define USART_ISR_ABRE      14
#define USART_ISR_ABRF      15
#define USART_ISR_BUSY      16
#define USART_ISR_CMF       17
#define USART_ISR_SBKF      18
#define USART_ISR_RWU       19
#define USART_ISR_WUF       20
#define USART_ISR_TEACK     21
#define USART_ISR_REACK     22
#define USART_ISR_TCBGT     25

#define USART_ICR_PECF      0
#define USART_ICR_FECF      1
#define USART_ICR_NCF       2
#define USART_ICR_ORECF     3
#define USART_ICR_IDLECF    4
#define USART_ICR_TCCF      6
#define USART_ICR_TCBGTCF   7
#define USART_ICR_LBDCF     8
#define USART_ICR_CTSCF     9
#define USART_ICR_RTOCF     11
#define USART_ICR_EOBCF     12
#define USART_ICR_CMCF      17
#define USART_ICR_WUCF      20


#include "STM32F767ZI_gpio_driver.h"
#include "STM32F767ZI_spi_driver.h"
#include "STM32F767ZI_i2c_driver.h"
#include "STM32F767ZI_usart_driver.h"
#include "STM32F767ZI_rcc_driver.h"

#endif /* INC_STM32F767ZI_H_ */
