################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/cli/cli_conf.c \
../Common/cli/cli_main.c \
../Common/cli/cli_pki.c \
../Common/cli/cli_rngtest.c \
../Common/cli/cli_uart_drv.c \
../Common/cli/cli_utils.c \
../Common/cli/logging.c 

OBJS += \
./Common/cli/cli_conf.o \
./Common/cli/cli_main.o \
./Common/cli/cli_pki.o \
./Common/cli/cli_rngtest.o \
./Common/cli/cli_uart_drv.o \
./Common/cli/cli_utils.o \
./Common/cli/logging.o 

C_DEPS += \
./Common/cli/cli_conf.d \
./Common/cli/cli_main.d \
./Common/cli/cli_pki.d \
./Common/cli/cli_rngtest.d \
./Common/cli/cli_uart_drv.d \
./Common/cli/cli_utils.d \
./Common/cli/logging.d 


# Each subdirectory must supply rules for building sources it contributes
Common/cli/%.o Common/cli/%.su Common/cli/%.cyclo: ../Common/cli/%.c Common/cli/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/cli/cli_conf.c_includes.args"

clean: clean-Common-2f-cli

clean-Common-2f-cli:
	-$(RM) ./Common/cli/cli_conf.cyclo ./Common/cli/cli_conf.d ./Common/cli/cli_conf.o ./Common/cli/cli_conf.su ./Common/cli/cli_main.cyclo ./Common/cli/cli_main.d ./Common/cli/cli_main.o ./Common/cli/cli_main.su ./Common/cli/cli_pki.cyclo ./Common/cli/cli_pki.d ./Common/cli/cli_pki.o ./Common/cli/cli_pki.su ./Common/cli/cli_rngtest.cyclo ./Common/cli/cli_rngtest.d ./Common/cli/cli_rngtest.o ./Common/cli/cli_rngtest.su ./Common/cli/cli_uart_drv.cyclo ./Common/cli/cli_uart_drv.d ./Common/cli/cli_uart_drv.o ./Common/cli/cli_uart_drv.su ./Common/cli/cli_utils.cyclo ./Common/cli/cli_utils.d ./Common/cli/cli_utils.o ./Common/cli/cli_utils.su ./Common/cli/logging.cyclo ./Common/cli/logging.d ./Common/cli/logging.o ./Common/cli/logging.su

.PHONY: clean-Common-2f-cli

