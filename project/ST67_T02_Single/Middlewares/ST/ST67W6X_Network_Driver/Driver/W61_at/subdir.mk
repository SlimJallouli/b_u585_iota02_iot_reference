################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.c \
../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.c 

OBJS += \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.o \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.o 

C_DEPS += \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.d \
./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/%.o Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/%.su Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/%.cyclo: ../Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/%.c Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.c_includes.args"

clean: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_at

clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_at:
	-$(RM) ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/modem_cmd_handler.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_ble.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_common.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_mqtt.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_net.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_sys.su ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.d ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.o ./Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/w61_at_wifi.su

.PHONY: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Driver-2f-W61_at

