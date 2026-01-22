################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FreeRTOS/freertos_hooks.c 

OBJS += \
./Core/Src/FreeRTOS/freertos_hooks.o 

C_DEPS += \
./Core/Src/FreeRTOS/freertos_hooks.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/FreeRTOS/%.o Core/Src/FreeRTOS/%.su Core/Src/FreeRTOS/%.cyclo: ../Core/Src/FreeRTOS/%.c Core/Src/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Core/Src/FreeRTOS/freertos_hooks.c_includes.args"

clean: clean-Core-2f-Src-2f-FreeRTOS

clean-Core-2f-Src-2f-FreeRTOS:
	-$(RM) ./Core/Src/FreeRTOS/freertos_hooks.cyclo ./Core/Src/FreeRTOS/freertos_hooks.d ./Core/Src/FreeRTOS/freertos_hooks.o ./Core/Src/FreeRTOS/freertos_hooks.su

.PHONY: clean-Core-2f-Src-2f-FreeRTOS

