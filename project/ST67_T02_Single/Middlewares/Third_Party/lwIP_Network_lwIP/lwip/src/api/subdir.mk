################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.c \
../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.c 

OBJS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.o \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.o 

C_DEPS += \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.d \
./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/%.o Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/%.su Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/%.cyclo: ../Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/%.c Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-api

clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-api:
	-$(RM) ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_lib.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/api_msg.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/err.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/if_api.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netbuf.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netdb.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/netifapi.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/sockets.su ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.cyclo ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.d ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.o ./Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/tcpip.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lwIP_Network_lwIP-2f-lwip-2f-src-2f-api

