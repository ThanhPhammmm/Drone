################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SensorDriver/BMI088/bmi088.c \
../Core/Src/SensorDriver/BMI088/bmi088_port.c \
../Core/Src/SensorDriver/BMI088/bmi088_task.c 

OBJS += \
./Core/Src/SensorDriver/BMI088/bmi088.o \
./Core/Src/SensorDriver/BMI088/bmi088_port.o \
./Core/Src/SensorDriver/BMI088/bmi088_task.o 

C_DEPS += \
./Core/Src/SensorDriver/BMI088/bmi088.d \
./Core/Src/SensorDriver/BMI088/bmi088_port.d \
./Core/Src/SensorDriver/BMI088/bmi088_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SensorDriver/BMI088/%.o Core/Src/SensorDriver/BMI088/%.su Core/Src/SensorDriver/BMI088/%.cyclo: ../Core/Src/SensorDriver/BMI088/%.c Core/Src/SensorDriver/BMI088/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SensorDriver-2f-BMI088

clean-Core-2f-Src-2f-SensorDriver-2f-BMI088:
	-$(RM) ./Core/Src/SensorDriver/BMI088/bmi088.cyclo ./Core/Src/SensorDriver/BMI088/bmi088.d ./Core/Src/SensorDriver/BMI088/bmi088.o ./Core/Src/SensorDriver/BMI088/bmi088.su ./Core/Src/SensorDriver/BMI088/bmi088_port.cyclo ./Core/Src/SensorDriver/BMI088/bmi088_port.d ./Core/Src/SensorDriver/BMI088/bmi088_port.o ./Core/Src/SensorDriver/BMI088/bmi088_port.su ./Core/Src/SensorDriver/BMI088/bmi088_task.cyclo ./Core/Src/SensorDriver/BMI088/bmi088_task.d ./Core/Src/SensorDriver/BMI088/bmi088_task.o ./Core/Src/SensorDriver/BMI088/bmi088_task.su

.PHONY: clean-Core-2f-Src-2f-SensorDriver-2f-BMI088

