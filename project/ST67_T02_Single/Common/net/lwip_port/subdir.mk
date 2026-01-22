################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/net/lwip_port/lwip_freertos.c \
../Common/net/lwip_port/mbedtls_transport.c 

OBJS += \
./Common/net/lwip_port/lwip_freertos.o \
./Common/net/lwip_port/mbedtls_transport.o 

C_DEPS += \
./Common/net/lwip_port/lwip_freertos.d \
./Common/net/lwip_port/mbedtls_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Common/net/lwip_port/%.o Common/net/lwip_port/%.su Common/net/lwip_port/%.cyclo: ../Common/net/lwip_port/%.c Common/net/lwip_port/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/net/lwip_port/lwip_freertos.c_includes.args"

clean: clean-Common-2f-net-2f-lwip_port

clean-Common-2f-net-2f-lwip_port:
	-$(RM) ./Common/net/lwip_port/lwip_freertos.cyclo ./Common/net/lwip_port/lwip_freertos.d ./Common/net/lwip_port/lwip_freertos.o ./Common/net/lwip_port/lwip_freertos.su ./Common/net/lwip_port/mbedtls_transport.cyclo ./Common/net/lwip_port/mbedtls_transport.d ./Common/net/lwip_port/mbedtls_transport.o ./Common/net/lwip_port/mbedtls_transport.su

.PHONY: clean-Common-2f-net-2f-lwip_port

