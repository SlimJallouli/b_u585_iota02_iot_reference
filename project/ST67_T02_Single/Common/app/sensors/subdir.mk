################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/sensors/env_sensor_publish.c \
../Common/app/sensors/motion_sensors_publish.c 

OBJS += \
./Common/app/sensors/env_sensor_publish.o \
./Common/app/sensors/motion_sensors_publish.o 

C_DEPS += \
./Common/app/sensors/env_sensor_publish.d \
./Common/app/sensors/motion_sensors_publish.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/sensors/%.o Common/app/sensors/%.su Common/app/sensors/%.cyclo: ../Common/app/sensors/%.c Common/app/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/sensors/env_sensor_publish.c_includes.args"

clean: clean-Common-2f-app-2f-sensors

clean-Common-2f-app-2f-sensors:
	-$(RM) ./Common/app/sensors/env_sensor_publish.cyclo ./Common/app/sensors/env_sensor_publish.d ./Common/app/sensors/env_sensor_publish.o ./Common/app/sensors/env_sensor_publish.su ./Common/app/sensors/motion_sensors_publish.cyclo ./Common/app/sensors/motion_sensors_publish.d ./Common/app/sensors/motion_sensors_publish.o ./Common/app/sensors/motion_sensors_publish.su

.PHONY: clean-Common-2f-app-2f-sensors

