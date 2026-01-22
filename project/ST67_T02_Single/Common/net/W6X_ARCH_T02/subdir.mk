################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/net/W6X_ARCH_T02/dhcp_server_raw.c \
../Common/net/W6X_ARCH_T02/lwip.c \
../Common/net/W6X_ARCH_T02/lwip_netif.c \
../Common/net/W6X_ARCH_T02/st67w6x_netconn.c 

OBJS += \
./Common/net/W6X_ARCH_T02/dhcp_server_raw.o \
./Common/net/W6X_ARCH_T02/lwip.o \
./Common/net/W6X_ARCH_T02/lwip_netif.o \
./Common/net/W6X_ARCH_T02/st67w6x_netconn.o 

C_DEPS += \
./Common/net/W6X_ARCH_T02/dhcp_server_raw.d \
./Common/net/W6X_ARCH_T02/lwip.d \
./Common/net/W6X_ARCH_T02/lwip_netif.d \
./Common/net/W6X_ARCH_T02/st67w6x_netconn.d 


# Each subdirectory must supply rules for building sources it contributes
Common/net/W6X_ARCH_T02/%.o Common/net/W6X_ARCH_T02/%.su Common/net/W6X_ARCH_T02/%.cyclo: ../Common/net/W6X_ARCH_T02/%.c Common/net/W6X_ARCH_T02/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/net/W6X_ARCH_T02/dhcp_server_raw.c_includes.args"

clean: clean-Common-2f-net-2f-W6X_ARCH_T02

clean-Common-2f-net-2f-W6X_ARCH_T02:
	-$(RM) ./Common/net/W6X_ARCH_T02/dhcp_server_raw.cyclo ./Common/net/W6X_ARCH_T02/dhcp_server_raw.d ./Common/net/W6X_ARCH_T02/dhcp_server_raw.o ./Common/net/W6X_ARCH_T02/dhcp_server_raw.su ./Common/net/W6X_ARCH_T02/lwip.cyclo ./Common/net/W6X_ARCH_T02/lwip.d ./Common/net/W6X_ARCH_T02/lwip.o ./Common/net/W6X_ARCH_T02/lwip.su ./Common/net/W6X_ARCH_T02/lwip_netif.cyclo ./Common/net/W6X_ARCH_T02/lwip_netif.d ./Common/net/W6X_ARCH_T02/lwip_netif.o ./Common/net/W6X_ARCH_T02/lwip_netif.su ./Common/net/W6X_ARCH_T02/st67w6x_netconn.cyclo ./Common/net/W6X_ARCH_T02/st67w6x_netconn.d ./Common/net/W6X_ARCH_T02/st67w6x_netconn.o ./Common/net/W6X_ARCH_T02/st67w6x_netconn.su

.PHONY: clean-Common-2f-net-2f-W6X_ARCH_T02

