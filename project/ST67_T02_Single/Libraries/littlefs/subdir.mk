################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/littlefs/lfs.c \
../Libraries/littlefs/lfs_util.c 

OBJS += \
./Libraries/littlefs/lfs.o \
./Libraries/littlefs/lfs_util.o 

C_DEPS += \
./Libraries/littlefs/lfs.d \
./Libraries/littlefs/lfs_util.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/littlefs/%.o Libraries/littlefs/%.su Libraries/littlefs/%.cyclo: ../Libraries/littlefs/%.c Libraries/littlefs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Libraries/littlefs/lfs.c_includes.args"

clean: clean-Libraries-2f-littlefs

clean-Libraries-2f-littlefs:
	-$(RM) ./Libraries/littlefs/lfs.cyclo ./Libraries/littlefs/lfs.d ./Libraries/littlefs/lfs.o ./Libraries/littlefs/lfs.su ./Libraries/littlefs/lfs_util.cyclo ./Libraries/littlefs/lfs_util.d ./Libraries/littlefs/lfs_util.o ./Libraries/littlefs/lfs_util.su

.PHONY: clean-Libraries-2f-littlefs

