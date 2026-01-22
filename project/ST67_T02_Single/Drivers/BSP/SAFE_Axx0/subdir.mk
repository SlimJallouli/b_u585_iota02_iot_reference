################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/SAFE_Axx0/stsafea_core.c \
../Drivers/BSP/SAFE_Axx0/stsafea_crc.c \
../Drivers/BSP/SAFE_Axx0/stsafea_crypto.c \
../Drivers/BSP/SAFE_Axx0/stsafea_service.c 

OBJS += \
./Drivers/BSP/SAFE_Axx0/stsafea_core.o \
./Drivers/BSP/SAFE_Axx0/stsafea_crc.o \
./Drivers/BSP/SAFE_Axx0/stsafea_crypto.o \
./Drivers/BSP/SAFE_Axx0/stsafea_service.o 

C_DEPS += \
./Drivers/BSP/SAFE_Axx0/stsafea_core.d \
./Drivers/BSP/SAFE_Axx0/stsafea_crc.d \
./Drivers/BSP/SAFE_Axx0/stsafea_crypto.d \
./Drivers/BSP/SAFE_Axx0/stsafea_service.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/SAFE_Axx0/%.o Drivers/BSP/SAFE_Axx0/%.su Drivers/BSP/SAFE_Axx0/%.cyclo: ../Drivers/BSP/SAFE_Axx0/%.c Drivers/BSP/SAFE_Axx0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Drivers/BSP/SAFE_Axx0/stsafea_core.c_includes.args"

clean: clean-Drivers-2f-BSP-2f-SAFE_Axx0

clean-Drivers-2f-BSP-2f-SAFE_Axx0:
	-$(RM) ./Drivers/BSP/SAFE_Axx0/stsafea_core.cyclo ./Drivers/BSP/SAFE_Axx0/stsafea_core.d ./Drivers/BSP/SAFE_Axx0/stsafea_core.o ./Drivers/BSP/SAFE_Axx0/stsafea_core.su ./Drivers/BSP/SAFE_Axx0/stsafea_crc.cyclo ./Drivers/BSP/SAFE_Axx0/stsafea_crc.d ./Drivers/BSP/SAFE_Axx0/stsafea_crc.o ./Drivers/BSP/SAFE_Axx0/stsafea_crc.su ./Drivers/BSP/SAFE_Axx0/stsafea_crypto.cyclo ./Drivers/BSP/SAFE_Axx0/stsafea_crypto.d ./Drivers/BSP/SAFE_Axx0/stsafea_crypto.o ./Drivers/BSP/SAFE_Axx0/stsafea_crypto.su ./Drivers/BSP/SAFE_Axx0/stsafea_service.cyclo ./Drivers/BSP/SAFE_Axx0/stsafea_service.d ./Drivers/BSP/SAFE_Axx0/stsafea_service.o ./Drivers/BSP/SAFE_Axx0/stsafea_service.su

.PHONY: clean-Drivers-2f-BSP-2f-SAFE_Axx0

