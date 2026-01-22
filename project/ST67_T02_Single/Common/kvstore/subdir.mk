################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/kvstore/kvstore.c \
../Common/kvstore/kvstore_cache.c \
../Common/kvstore/kvstore_nv_littlefs.c \
../Common/kvstore/kvstore_nv_psa_its.c \
../Common/kvstore/kvstore_nv_stsafe.c 

OBJS += \
./Common/kvstore/kvstore.o \
./Common/kvstore/kvstore_cache.o \
./Common/kvstore/kvstore_nv_littlefs.o \
./Common/kvstore/kvstore_nv_psa_its.o \
./Common/kvstore/kvstore_nv_stsafe.o 

C_DEPS += \
./Common/kvstore/kvstore.d \
./Common/kvstore/kvstore_cache.d \
./Common/kvstore/kvstore_nv_littlefs.d \
./Common/kvstore/kvstore_nv_psa_its.d \
./Common/kvstore/kvstore_nv_stsafe.d 


# Each subdirectory must supply rules for building sources it contributes
Common/kvstore/%.o Common/kvstore/%.su Common/kvstore/%.cyclo: ../Common/kvstore/%.c Common/kvstore/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/kvstore/kvstore.c_includes.args"

clean: clean-Common-2f-kvstore

clean-Common-2f-kvstore:
	-$(RM) ./Common/kvstore/kvstore.cyclo ./Common/kvstore/kvstore.d ./Common/kvstore/kvstore.o ./Common/kvstore/kvstore.su ./Common/kvstore/kvstore_cache.cyclo ./Common/kvstore/kvstore_cache.d ./Common/kvstore/kvstore_cache.o ./Common/kvstore/kvstore_cache.su ./Common/kvstore/kvstore_nv_littlefs.cyclo ./Common/kvstore/kvstore_nv_littlefs.d ./Common/kvstore/kvstore_nv_littlefs.o ./Common/kvstore/kvstore_nv_littlefs.su ./Common/kvstore/kvstore_nv_psa_its.cyclo ./Common/kvstore/kvstore_nv_psa_its.d ./Common/kvstore/kvstore_nv_psa_its.o ./Common/kvstore/kvstore_nv_psa_its.su ./Common/kvstore/kvstore_nv_stsafe.cyclo ./Common/kvstore/kvstore_nv_stsafe.d ./Common/kvstore/kvstore_nv_stsafe.o ./Common/kvstore/kvstore_nv_stsafe.su

.PHONY: clean-Common-2f-kvstore

