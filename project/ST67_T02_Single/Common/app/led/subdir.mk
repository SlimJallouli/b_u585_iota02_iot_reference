################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/led/led_task.c 

OBJS += \
./Common/app/led/led_task.o 

C_DEPS += \
./Common/app/led/led_task.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/led/%.o Common/app/led/%.su Common/app/led/%.cyclo: ../Common/app/led/%.c Common/app/led/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/led/led_task.c_includes.args"

clean: clean-Common-2f-app-2f-led

clean-Common-2f-app-2f-led:
	-$(RM) ./Common/app/led/led_task.cyclo ./Common/app/led/led_task.d ./Common/app/led/led_task.o ./Common/app/led/led_task.su

.PHONY: clean-Common-2f-app-2f-led

