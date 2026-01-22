################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lps22hh/lps22hh.c \
../Drivers/BSP/Components/lps22hh/lps22hh_reg.c 

OBJS += \
./Drivers/BSP/Components/lps22hh/lps22hh.o \
./Drivers/BSP/Components/lps22hh/lps22hh_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lps22hh/lps22hh.d \
./Drivers/BSP/Components/lps22hh/lps22hh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lps22hh/%.o Drivers/BSP/Components/lps22hh/%.su Drivers/BSP/Components/lps22hh/%.cyclo: ../Drivers/BSP/Components/lps22hh/%.c Drivers/BSP/Components/lps22hh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Drivers/BSP/Components/lps22hh/lps22hh.c_includes.args"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lps22hh

clean-Drivers-2f-BSP-2f-Components-2f-lps22hh:
	-$(RM) ./Drivers/BSP/Components/lps22hh/lps22hh.cyclo ./Drivers/BSP/Components/lps22hh/lps22hh.d ./Drivers/BSP/Components/lps22hh/lps22hh.o ./Drivers/BSP/Components/lps22hh/lps22hh.su ./Drivers/BSP/Components/lps22hh/lps22hh_reg.cyclo ./Drivers/BSP/Components/lps22hh/lps22hh_reg.d ./Drivers/BSP/Components/lps22hh/lps22hh_reg.o ./Drivers/BSP/Components/lps22hh/lps22hh_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lps22hh

