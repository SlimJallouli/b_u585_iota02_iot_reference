################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/crypto/core_pkcs11_pal_littlefs.c \
../Core/Src/crypto/core_pkcs11_pal_stsafe.c \
../Core/Src/crypto/core_pkcs11_pal_utils.c \
../Core/Src/crypto/hardware_rng.c 

OBJS += \
./Core/Src/crypto/core_pkcs11_pal_littlefs.o \
./Core/Src/crypto/core_pkcs11_pal_stsafe.o \
./Core/Src/crypto/core_pkcs11_pal_utils.o \
./Core/Src/crypto/hardware_rng.o 

C_DEPS += \
./Core/Src/crypto/core_pkcs11_pal_littlefs.d \
./Core/Src/crypto/core_pkcs11_pal_stsafe.d \
./Core/Src/crypto/core_pkcs11_pal_utils.d \
./Core/Src/crypto/hardware_rng.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/crypto/%.o Core/Src/crypto/%.su Core/Src/crypto/%.cyclo: ../Core/Src/crypto/%.c Core/Src/crypto/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Core/Src/crypto/core_pkcs11_pal_littlefs.c_includes.args"

clean: clean-Core-2f-Src-2f-crypto

clean-Core-2f-Src-2f-crypto:
	-$(RM) ./Core/Src/crypto/core_pkcs11_pal_littlefs.cyclo ./Core/Src/crypto/core_pkcs11_pal_littlefs.d ./Core/Src/crypto/core_pkcs11_pal_littlefs.o ./Core/Src/crypto/core_pkcs11_pal_littlefs.su ./Core/Src/crypto/core_pkcs11_pal_stsafe.cyclo ./Core/Src/crypto/core_pkcs11_pal_stsafe.d ./Core/Src/crypto/core_pkcs11_pal_stsafe.o ./Core/Src/crypto/core_pkcs11_pal_stsafe.su ./Core/Src/crypto/core_pkcs11_pal_utils.cyclo ./Core/Src/crypto/core_pkcs11_pal_utils.d ./Core/Src/crypto/core_pkcs11_pal_utils.o ./Core/Src/crypto/core_pkcs11_pal_utils.su ./Core/Src/crypto/hardware_rng.cyclo ./Core/Src/crypto/hardware_rng.d ./Core/Src/crypto/hardware_rng.o ./Core/Src/crypto/hardware_rng.su

.PHONY: clean-Core-2f-Src-2f-crypto

