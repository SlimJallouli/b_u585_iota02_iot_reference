################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.c \
../Common/app/aws/FleetProvisioning/pkcs11_operations.c \
../Common/app/aws/FleetProvisioning/tinycbor_serializer.c 

OBJS += \
./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.o \
./Common/app/aws/FleetProvisioning/pkcs11_operations.o \
./Common/app/aws/FleetProvisioning/tinycbor_serializer.o 

C_DEPS += \
./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.d \
./Common/app/aws/FleetProvisioning/pkcs11_operations.d \
./Common/app/aws/FleetProvisioning/tinycbor_serializer.d 


# Each subdirectory must supply rules for building sources it contributes
Common/app/aws/FleetProvisioning/%.o Common/app/aws/FleetProvisioning/%.su Common/app/aws/FleetProvisioning/%.cyclo: ../Common/app/aws/FleetProvisioning/%.c Common/app/aws/FleetProvisioning/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.c_includes.args"

clean: clean-Common-2f-app-2f-aws-2f-FleetProvisioning

clean-Common-2f-app-2f-aws-2f-FleetProvisioning:
	-$(RM) ./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.cyclo ./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.d ./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.o ./Common/app/aws/FleetProvisioning/fleet_provisioning_task_cbor.su ./Common/app/aws/FleetProvisioning/pkcs11_operations.cyclo ./Common/app/aws/FleetProvisioning/pkcs11_operations.d ./Common/app/aws/FleetProvisioning/pkcs11_operations.o ./Common/app/aws/FleetProvisioning/pkcs11_operations.su ./Common/app/aws/FleetProvisioning/tinycbor_serializer.cyclo ./Common/app/aws/FleetProvisioning/tinycbor_serializer.d ./Common/app/aws/FleetProvisioning/tinycbor_serializer.o ./Common/app/aws/FleetProvisioning/tinycbor_serializer.su

.PHONY: clean-Common-2f-app-2f-aws-2f-FleetProvisioning

