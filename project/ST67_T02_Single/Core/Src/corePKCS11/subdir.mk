################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/corePKCS11/core_pkcs11_mbedtls.c 

OBJS += \
./Core/Src/corePKCS11/core_pkcs11_mbedtls.o 

C_DEPS += \
./Core/Src/corePKCS11/core_pkcs11_mbedtls.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/corePKCS11/%.o Core/Src/corePKCS11/%.su Core/Src/corePKCS11/%.cyclo: ../Core/Src/corePKCS11/%.c Core/Src/corePKCS11/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Core/Src/corePKCS11/core_pkcs11_mbedtls.c_includes.args"

clean: clean-Core-2f-Src-2f-corePKCS11

clean-Core-2f-Src-2f-corePKCS11:
	-$(RM) ./Core/Src/corePKCS11/core_pkcs11_mbedtls.cyclo ./Core/Src/corePKCS11/core_pkcs11_mbedtls.d ./Core/Src/corePKCS11/core_pkcs11_mbedtls.o ./Core/Src/corePKCS11/core_pkcs11_mbedtls.su

.PHONY: clean-Core-2f-Src-2f-corePKCS11

