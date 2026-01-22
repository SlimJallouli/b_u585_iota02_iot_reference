################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/sys/interrupt_handlers.c \
../Common/sys/newlibc_stubs.c 

OBJS += \
./Common/sys/interrupt_handlers.o \
./Common/sys/newlibc_stubs.o 

C_DEPS += \
./Common/sys/interrupt_handlers.d \
./Common/sys/newlibc_stubs.d 


# Each subdirectory must supply rules for building sources it contributes
Common/sys/%.o Common/sys/%.su Common/sys/%.cyclo: ../Common/sys/%.c Common/sys/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/sys/interrupt_handlers.c_includes.args"

clean: clean-Common-2f-sys

clean-Common-2f-sys:
	-$(RM) ./Common/sys/interrupt_handlers.cyclo ./Common/sys/interrupt_handlers.d ./Common/sys/interrupt_handlers.o ./Common/sys/interrupt_handlers.su ./Common/sys/newlibc_stubs.cyclo ./Common/sys/newlibc_stubs.d ./Common/sys/newlibc_stubs.o ./Common/sys/newlibc_stubs.su

.PHONY: clean-Common-2f-sys

