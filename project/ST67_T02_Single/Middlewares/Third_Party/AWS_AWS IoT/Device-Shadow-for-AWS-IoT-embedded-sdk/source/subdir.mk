################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.c 

OBJS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.o 

C_DEPS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.c Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-Device-2d-Shadow-2d-for-2d-AWS-2d-IoT-2d-embedded-2d-sdk-2f-source

clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-Device-2d-Shadow-2d-for-2d-AWS-2d-IoT-2d-embedded-2d-sdk-2f-source:
	-$(RM) ./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.d ./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.o ./Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/shadow.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-Device-2d-Shadow-2d-for-2d-AWS-2d-IoT-2d-embedded-2d-sdk-2f-source

