################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/tinycbor/src/cborencoder.c \
../Libraries/tinycbor/src/cborencoder_close_container_checked.c \
../Libraries/tinycbor/src/cborencoder_float.c \
../Libraries/tinycbor/src/cborerrorstrings.c \
../Libraries/tinycbor/src/cborparser.c \
../Libraries/tinycbor/src/cborparser_dup_string.c \
../Libraries/tinycbor/src/cborparser_float.c \
../Libraries/tinycbor/src/cborpretty.c \
../Libraries/tinycbor/src/cborpretty_stdio.c \
../Libraries/tinycbor/src/cbortojson.c \
../Libraries/tinycbor/src/cborvalidation.c 

OBJS += \
./Libraries/tinycbor/src/cborencoder.o \
./Libraries/tinycbor/src/cborencoder_close_container_checked.o \
./Libraries/tinycbor/src/cborencoder_float.o \
./Libraries/tinycbor/src/cborerrorstrings.o \
./Libraries/tinycbor/src/cborparser.o \
./Libraries/tinycbor/src/cborparser_dup_string.o \
./Libraries/tinycbor/src/cborparser_float.o \
./Libraries/tinycbor/src/cborpretty.o \
./Libraries/tinycbor/src/cborpretty_stdio.o \
./Libraries/tinycbor/src/cbortojson.o \
./Libraries/tinycbor/src/cborvalidation.o 

C_DEPS += \
./Libraries/tinycbor/src/cborencoder.d \
./Libraries/tinycbor/src/cborencoder_close_container_checked.d \
./Libraries/tinycbor/src/cborencoder_float.d \
./Libraries/tinycbor/src/cborerrorstrings.d \
./Libraries/tinycbor/src/cborparser.d \
./Libraries/tinycbor/src/cborparser_dup_string.d \
./Libraries/tinycbor/src/cborparser_float.d \
./Libraries/tinycbor/src/cborpretty.d \
./Libraries/tinycbor/src/cborpretty_stdio.d \
./Libraries/tinycbor/src/cbortojson.d \
./Libraries/tinycbor/src/cborvalidation.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/tinycbor/src/%.o Libraries/tinycbor/src/%.su Libraries/tinycbor/src/%.cyclo: ../Libraries/tinycbor/src/%.c Libraries/tinycbor/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Libraries/tinycbor/src/cborencoder.c_includes.args"

clean: clean-Libraries-2f-tinycbor-2f-src

clean-Libraries-2f-tinycbor-2f-src:
	-$(RM) ./Libraries/tinycbor/src/cborencoder.cyclo ./Libraries/tinycbor/src/cborencoder.d ./Libraries/tinycbor/src/cborencoder.o ./Libraries/tinycbor/src/cborencoder.su ./Libraries/tinycbor/src/cborencoder_close_container_checked.cyclo ./Libraries/tinycbor/src/cborencoder_close_container_checked.d ./Libraries/tinycbor/src/cborencoder_close_container_checked.o ./Libraries/tinycbor/src/cborencoder_close_container_checked.su ./Libraries/tinycbor/src/cborencoder_float.cyclo ./Libraries/tinycbor/src/cborencoder_float.d ./Libraries/tinycbor/src/cborencoder_float.o ./Libraries/tinycbor/src/cborencoder_float.su ./Libraries/tinycbor/src/cborerrorstrings.cyclo ./Libraries/tinycbor/src/cborerrorstrings.d ./Libraries/tinycbor/src/cborerrorstrings.o ./Libraries/tinycbor/src/cborerrorstrings.su ./Libraries/tinycbor/src/cborparser.cyclo ./Libraries/tinycbor/src/cborparser.d ./Libraries/tinycbor/src/cborparser.o ./Libraries/tinycbor/src/cborparser.su ./Libraries/tinycbor/src/cborparser_dup_string.cyclo ./Libraries/tinycbor/src/cborparser_dup_string.d ./Libraries/tinycbor/src/cborparser_dup_string.o ./Libraries/tinycbor/src/cborparser_dup_string.su ./Libraries/tinycbor/src/cborparser_float.cyclo ./Libraries/tinycbor/src/cborparser_float.d ./Libraries/tinycbor/src/cborparser_float.o ./Libraries/tinycbor/src/cborparser_float.su ./Libraries/tinycbor/src/cborpretty.cyclo ./Libraries/tinycbor/src/cborpretty.d ./Libraries/tinycbor/src/cborpretty.o ./Libraries/tinycbor/src/cborpretty.su ./Libraries/tinycbor/src/cborpretty_stdio.cyclo ./Libraries/tinycbor/src/cborpretty_stdio.d ./Libraries/tinycbor/src/cborpretty_stdio.o ./Libraries/tinycbor/src/cborpretty_stdio.su ./Libraries/tinycbor/src/cbortojson.cyclo ./Libraries/tinycbor/src/cbortojson.d ./Libraries/tinycbor/src/cbortojson.o ./Libraries/tinycbor/src/cbortojson.su ./Libraries/tinycbor/src/cborvalidation.cyclo ./Libraries/tinycbor/src/cborvalidation.d ./Libraries/tinycbor/src/cborvalidation.o ./Libraries/tinycbor/src/cborvalidation.su

.PHONY: clean-Libraries-2f-tinycbor-2f-src

