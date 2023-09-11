################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Custom_Driver/Src/Custom_I2C.c 

OBJS += \
./Drivers/Custom_Driver/Src/Custom_I2C.o 

C_DEPS += \
./Drivers/Custom_Driver/Src/Custom_I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Custom_Driver/Src/%.o Drivers/Custom_Driver/Src/%.su Drivers/Custom_Driver/Src/%.cyclo: ../Drivers/Custom_Driver/Src/%.c Drivers/Custom_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/EMBEDDED C/BOOTLOADER/I2C_01_260623/Drivers/Custom_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Custom_Driver-2f-Src

clean-Drivers-2f-Custom_Driver-2f-Src:
	-$(RM) ./Drivers/Custom_Driver/Src/Custom_I2C.cyclo ./Drivers/Custom_Driver/Src/Custom_I2C.d ./Drivers/Custom_Driver/Src/Custom_I2C.o ./Drivers/Custom_Driver/Src/Custom_I2C.su

.PHONY: clean-Drivers-2f-Custom_Driver-2f-Src

