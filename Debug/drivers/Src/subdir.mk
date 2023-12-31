################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/STM32F767ZI_gpio_driver.c \
../drivers/Src/STM32F767ZI_i2c_driver.c \
../drivers/Src/STM32F767ZI_rcc_driver.c \
../drivers/Src/STM32F767ZI_spi_driver.c \
../drivers/Src/STM32F767ZI_usart_driver.c 

OBJS += \
./drivers/Src/STM32F767ZI_gpio_driver.o \
./drivers/Src/STM32F767ZI_i2c_driver.o \
./drivers/Src/STM32F767ZI_rcc_driver.o \
./drivers/Src/STM32F767ZI_spi_driver.o \
./drivers/Src/STM32F767ZI_usart_driver.o 

C_DEPS += \
./drivers/Src/STM32F767ZI_gpio_driver.d \
./drivers/Src/STM32F767ZI_i2c_driver.d \
./drivers/Src/STM32F767ZI_rcc_driver.d \
./drivers/Src/STM32F767ZI_spi_driver.d \
./drivers/Src/STM32F767ZI_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DSTM32F767ZITx -c -I../Inc -I"C:/Users/rahim/Desktop/Udemy/Mastering Microcontroller and Embedded Driver Development/Coding_Excercises/STM32F767ZI_Driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/STM32F767ZI_gpio_driver.cyclo ./drivers/Src/STM32F767ZI_gpio_driver.d ./drivers/Src/STM32F767ZI_gpio_driver.o ./drivers/Src/STM32F767ZI_gpio_driver.su ./drivers/Src/STM32F767ZI_i2c_driver.cyclo ./drivers/Src/STM32F767ZI_i2c_driver.d ./drivers/Src/STM32F767ZI_i2c_driver.o ./drivers/Src/STM32F767ZI_i2c_driver.su ./drivers/Src/STM32F767ZI_rcc_driver.cyclo ./drivers/Src/STM32F767ZI_rcc_driver.d ./drivers/Src/STM32F767ZI_rcc_driver.o ./drivers/Src/STM32F767ZI_rcc_driver.su ./drivers/Src/STM32F767ZI_spi_driver.cyclo ./drivers/Src/STM32F767ZI_spi_driver.d ./drivers/Src/STM32F767ZI_spi_driver.o ./drivers/Src/STM32F767ZI_spi_driver.su ./drivers/Src/STM32F767ZI_usart_driver.cyclo ./drivers/Src/STM32F767ZI_usart_driver.d ./drivers/Src/STM32F767ZI_usart_driver.o ./drivers/Src/STM32F767ZI_usart_driver.su

.PHONY: clean-drivers-2f-Src

