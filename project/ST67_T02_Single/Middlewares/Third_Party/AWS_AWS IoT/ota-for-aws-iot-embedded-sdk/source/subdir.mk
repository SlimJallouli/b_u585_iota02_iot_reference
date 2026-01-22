################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.c \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.c \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.c \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.c \
../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.c 

OBJS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.o \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.o \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.o \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.o \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.o 

C_DEPS += \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.d \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.d \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.d \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.d \
./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota.c_includes.args"
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.c_includes.args"
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.c_includes.args"
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.c_includes.args"
Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.o: ../Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.c Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/AWS_AWS IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source

clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source:
	-$(RM) ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota.su ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_base64.su ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_cbor.su ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_interface.su ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.cyclo ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.d ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.o ./Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/ota_mqtt.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-AWS_AWS-20-IoT-2f-ota-2d-for-2d-aws-2d-iot-2d-embedded-2d-sdk-2f-source

