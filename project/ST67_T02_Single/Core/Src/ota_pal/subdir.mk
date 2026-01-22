################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ota_pal/ota_firmware_version.c \
../Core/Src/ota_pal/ota_pal_stm32_ntz.c 

OBJS += \
./Core/Src/ota_pal/ota_firmware_version.o \
./Core/Src/ota_pal/ota_pal_stm32_ntz.o 

C_DEPS += \
./Core/Src/ota_pal/ota_firmware_version.d \
./Core/Src/ota_pal/ota_pal_stm32_ntz.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ota_pal/%.o Core/Src/ota_pal/%.su Core/Src/ota_pal/%.cyclo: ../Core/Src/ota_pal/%.c Core/Src/ota_pal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Core/Src/ota_pal/ota_firmware_version.c_includes.args"

clean: clean-Core-2f-Src-2f-ota_pal

clean-Core-2f-Src-2f-ota_pal:
	-$(RM) ./Core/Src/ota_pal/ota_firmware_version.cyclo ./Core/Src/ota_pal/ota_firmware_version.d ./Core/Src/ota_pal/ota_firmware_version.o ./Core/Src/ota_pal/ota_firmware_version.su ./Core/Src/ota_pal/ota_pal_stm32_ntz.cyclo ./Core/Src/ota_pal/ota_pal_stm32_ntz.d ./Core/Src/ota_pal/ota_pal_stm32_ntz.o ./Core/Src/ota_pal/ota_pal_stm32_ntz.su

.PHONY: clean-Core-2f-Src-2f-ota_pal

