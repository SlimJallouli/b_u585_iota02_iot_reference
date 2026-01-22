################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/aws/shadow/shadow_device_task.c 

OBJS += \
./Common/app/aws/shadow/shadow_device_task.o 

C_DEPS += \
./Common/app/aws/shadow/shadow_device_task.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/aws/shadow/%.o Common/app/aws/shadow/%.su Common/app/aws/shadow/%.cyclo: ../Common/app/aws/shadow/%.c Common/app/aws/shadow/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/aws/shadow/shadow_device_task.c_includes.args"

clean: clean-Common-2f-app-2f-aws-2f-shadow

clean-Common-2f-app-2f-aws-2f-shadow:
	-$(RM) ./Common/app/aws/shadow/shadow_device_task.cyclo ./Common/app/aws/shadow/shadow_device_task.d ./Common/app/aws/shadow/shadow_device_task.o ./Common/app/aws/shadow/shadow_device_task.su

.PHONY: clean-Common-2f-app-2f-aws-2f-shadow

