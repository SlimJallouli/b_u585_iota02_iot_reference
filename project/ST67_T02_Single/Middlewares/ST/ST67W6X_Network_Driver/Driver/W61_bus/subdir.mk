################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.c 

OBJS += \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.o 

C_DEPS += \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/%.o Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/%.su Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/%.cyclo: ../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/%.c Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.c_includes.args"

clean: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_bus

clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_bus:
	-$(RM) ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/spi_iface.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/w61_io.su

.PHONY: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_bus

