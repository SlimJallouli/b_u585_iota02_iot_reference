################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ST67W6X_Network_Driver/Target/spi_port.c 

OBJS += \
./ST67W6X_Network_Driver/Target/spi_port.o 

C_DEPS += \
./ST67W6X_Network_Driver/Target/spi_port.d 


# Each subdirectory must supply rules for building sources it contributes
ST67W6X_Network_Driver/Target/%.o ST67W6X_Network_Driver/Target/%.su ST67W6X_Network_Driver/Target/%.cyclo: ../ST67W6X_Network_Driver/Target/%.c ST67W6X_Network_Driver/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"ST67W6X_Network_Driver/Target/spi_port.c_includes.args"

clean: clean-ST67W6X_Network_Driver-2f-Target

clean-ST67W6X_Network_Driver-2f-Target:
	-$(RM) ./ST67W6X_Network_Driver/Target/spi_port.cyclo ./ST67W6X_Network_Driver/Target/spi_port.d ./ST67W6X_Network_Driver/Target/spi_port.o ./ST67W6X_Network_Driver/Target/spi_port.su

.PHONY: clean-ST67W6X_Network_Driver-2f-Target

