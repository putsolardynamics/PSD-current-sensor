################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/canopen_object_dict.c 

C_DEPS += \
./Core/Inc/canopen_object_dict.d 

OBJS += \
./Core/Inc/canopen_object_dict.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/canopen_object_dict.o: ../Core/Inc/canopen_object_dict.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/canopen_object_dict.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

