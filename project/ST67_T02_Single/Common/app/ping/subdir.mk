################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/ping/ping.c 

OBJS += \
./Common/app/ping/ping.o 

C_DEPS += \
./Common/app/ping/ping.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/ping/%.o Common/app/ping/%.su Common/app/ping/%.cyclo: ../Common/app/ping/%.c Common/app/ping/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/ping/ping.c_includes.args"

clean: clean-Common-2f-app-2f-ping

clean-Common-2f-app-2f-ping:
	-$(RM) ./Common/app/ping/ping.cyclo ./Common/app/ping/ping.d ./Common/app/ping/ping.o ./Common/app/ping/ping.su

.PHONY: clean-Common-2f-app-2f-ping

