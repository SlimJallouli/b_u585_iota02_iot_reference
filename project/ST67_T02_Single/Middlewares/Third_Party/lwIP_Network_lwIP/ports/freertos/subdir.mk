################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.c 

OBJS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.o 

C_DEPS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/%.o Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/%.su Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/%.cyclo: ../Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/%.c Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-ports-2f-freertos

clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-ports-2f-freertos:
	-$(RM) ./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.d ./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.o ./Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/sys_arch.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-ports-2f-freertos

