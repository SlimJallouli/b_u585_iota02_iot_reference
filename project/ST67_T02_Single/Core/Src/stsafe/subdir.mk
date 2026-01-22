################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stsafe/stsafe.c \
../Core/Src/stsafe/stsafe_crypto_interface.c \
../Core/Src/stsafe/stsafe_key_value_store.c \
../Core/Src/stsafe/stsafea_mbedtls_interface.c 

OBJS += \
./Core/Src/stsafe/stsafe.o \
./Core/Src/stsafe/stsafe_crypto_interface.o \
./Core/Src/stsafe/stsafe_key_value_store.o \
./Core/Src/stsafe/stsafea_mbedtls_interface.o 

C_DEPS += \
./Core/Src/stsafe/stsafe.d \
./Core/Src/stsafe/stsafe_crypto_interface.d \
./Core/Src/stsafe/stsafe_key_value_store.d \
./Core/Src/stsafe/stsafea_mbedtls_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/stsafe/%.o Core/Src/stsafe/%.su Core/Src/stsafe/%.cyclo: ../Core/Src/stsafe/%.c Core/Src/stsafe/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Core/Src/stsafe/stsafe.c_includes.args"

clean: clean-Core-2f-Src-2f-stsafe

clean-Core-2f-Src-2f-stsafe:
	-$(RM) ./Core/Src/stsafe/stsafe.cyclo ./Core/Src/stsafe/stsafe.d ./Core/Src/stsafe/stsafe.o ./Core/Src/stsafe/stsafe.su ./Core/Src/stsafe/stsafe_crypto_interface.cyclo ./Core/Src/stsafe/stsafe_crypto_interface.d ./Core/Src/stsafe/stsafe_crypto_interface.o ./Core/Src/stsafe/stsafe_crypto_interface.su ./Core/Src/stsafe/stsafe_key_value_store.cyclo ./Core/Src/stsafe/stsafe_key_value_store.d ./Core/Src/stsafe/stsafe_key_value_store.o ./Core/Src/stsafe/stsafe_key_value_store.su ./Core/Src/stsafe/stsafea_mbedtls_interface.cyclo ./Core/Src/stsafe/stsafea_mbedtls_interface.d ./Core/Src/stsafe/stsafea_mbedtls_interface.o ./Core/Src/stsafe/stsafea_mbedtls_interface.su

.PHONY: clean-Core-2f-Src-2f-stsafe

