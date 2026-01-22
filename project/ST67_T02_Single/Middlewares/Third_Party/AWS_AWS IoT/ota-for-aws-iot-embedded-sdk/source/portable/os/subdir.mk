################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.c 

OBJS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.o 

C_DEPS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source-2f-portable-2f-os

clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source-2f-portable-2f-os:
	-$(RM) ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/ota_os_freertos.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source-2f-portable-2f-os

