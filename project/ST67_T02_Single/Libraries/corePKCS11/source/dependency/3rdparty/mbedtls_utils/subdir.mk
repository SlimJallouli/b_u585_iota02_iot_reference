################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.c 

OBJS += \
./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.o 

C_DEPS += \
./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/%.o Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/%.su Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/%.cyclo: ../Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/%.c Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.c_includes.args"

clean: clean-Libraries-2f-corePKCS11-2f-source-2f-dependency-2f-3rdparty-2f-mbedtls_utils

clean-Libraries-2f-corePKCS11-2f-source-2f-dependency-2f-3rdparty-2f-mbedtls_utils:
	-$(RM) ./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.cyclo ./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.d ./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.o ./Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/mbedtls_utils.su

.PHONY: clean-Libraries-2f-corePKCS11-2f-source-2f-dependency-2f-3rdparty-2f-mbedtls_utils

