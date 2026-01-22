################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.c 

OBJS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.o 

C_DEPS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/%.o Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/%.su Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/%.cyclo: ../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/%.c Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-netif

clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-netif:
	-$(RM) ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/ethernet.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-netif

