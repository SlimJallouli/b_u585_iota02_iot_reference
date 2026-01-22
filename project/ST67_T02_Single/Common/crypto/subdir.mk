################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/crypto/PkiObject.c \
../Common/crypto/PkiObjectPkcs11.c \
../Common/crypto/PkiObjectPsa.c \
../Common/crypto/mbedtls_freertos_port.c \
../Common/crypto/mbedtls_pk_pkcs11.c \
../Common/crypto/mbedtls_pk_pkcs11_stsafe.c \
../Common/crypto/mbedtls_pk_psa.c \
../Common/crypto/pk_wrap_stsafe.c \
../Common/crypto/psa_util.c 

OBJS += \
./Common/crypto/PkiObject.o \
./Common/crypto/PkiObjectPkcs11.o \
./Common/crypto/PkiObjectPsa.o \
./Common/crypto/mbedtls_freertos_port.o \
./Common/crypto/mbedtls_pk_pkcs11.o \
./Common/crypto/mbedtls_pk_pkcs11_stsafe.o \
./Common/crypto/mbedtls_pk_psa.o \
./Common/crypto/pk_wrap_stsafe.o \
./Common/crypto/psa_util.o 

C_DEPS += \
./Common/crypto/PkiObject.d \
./Common/crypto/PkiObjectPkcs11.d \
./Common/crypto/PkiObjectPsa.d \
./Common/crypto/mbedtls_freertos_port.d \
./Common/crypto/mbedtls_pk_pkcs11.d \
./Common/crypto/mbedtls_pk_pkcs11_stsafe.d \
./Common/crypto/mbedtls_pk_psa.d \
./Common/crypto/pk_wrap_stsafe.d \
./Common/crypto/psa_util.d 


# Each subdirectory must supply rules for building sources it contributes
Common/crypto/%.o Common/crypto/%.su Common/crypto/%.cyclo: ../Common/crypto/%.c Common/crypto/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Common/crypto/PkiObject.c_includes.args"

clean: clean-Common-2f-crypto

clean-Common-2f-crypto:
	-$(RM) ./Common/crypto/PkiObject.cyclo ./Common/crypto/PkiObject.d ./Common/crypto/PkiObject.o ./Common/crypto/PkiObject.su ./Common/crypto/PkiObjectPkcs11.cyclo ./Common/crypto/PkiObjectPkcs11.d ./Common/crypto/PkiObjectPkcs11.o ./Common/crypto/PkiObjectPkcs11.su ./Common/crypto/PkiObjectPsa.cyclo ./Common/crypto/PkiObjectPsa.d ./Common/crypto/PkiObjectPsa.o ./Common/crypto/PkiObjectPsa.su ./Common/crypto/mbedtls_freertos_port.cyclo ./Common/crypto/mbedtls_freertos_port.d ./Common/crypto/mbedtls_freertos_port.o ./Common/crypto/mbedtls_freertos_port.su ./Common/crypto/mbedtls_pk_pkcs11.cyclo ./Common/crypto/mbedtls_pk_pkcs11.d ./Common/crypto/mbedtls_pk_pkcs11.o ./Common/crypto/mbedtls_pk_pkcs11.su ./Common/crypto/mbedtls_pk_pkcs11_stsafe.cyclo ./Common/crypto/mbedtls_pk_pkcs11_stsafe.d ./Common/crypto/mbedtls_pk_pkcs11_stsafe.o ./Common/crypto/mbedtls_pk_pkcs11_stsafe.su ./Common/crypto/mbedtls_pk_psa.cyclo ./Common/crypto/mbedtls_pk_psa.d ./Common/crypto/mbedtls_pk_psa.o ./Common/crypto/mbedtls_pk_psa.su ./Common/crypto/pk_wrap_stsafe.cyclo ./Common/crypto/pk_wrap_stsafe.d ./Common/crypto/pk_wrap_stsafe.o ./Common/crypto/pk_wrap_stsafe.su ./Common/crypto/psa_util.cyclo ./Common/crypto/psa_util.d ./Common/crypto/psa_util.o ./Common/crypto/psa_util.su

.PHONY: clean-Common-2f-crypto

