################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/button/button_task.c 

OBJS += \
./Common/app/button/button_task.o 

C_DEPS += \
./Common/app/button/button_task.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/button/%.o Common/app/button/%.su Common/app/button/%.cyclo: ../Common/app/button/%.c Common/app/button/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/button/button_task.c_includes.args"

clean: clean-Common-2f-app-2f-button

clean-Common-2f-app-2f-button:
	-$(RM) ./Common/app/button/button_task.cyclo ./Common/app/button/button_task.d ./Common/app/button/button_task.o ./Common/app/button/button_task.su

.PHONY: clean-Common-2f-app-2f-button

