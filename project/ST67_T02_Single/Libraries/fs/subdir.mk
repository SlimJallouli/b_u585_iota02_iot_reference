################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/fs/lfs_port_internal_nor_stm32h5.c \
../Libraries/fs/lfs_port_internal_nor_stm32u5.c \
../Libraries/fs/lfs_port_ospi.c \
../Libraries/fs/lfs_port_prv.c \
../Libraries/fs/lfs_port_xspi.c \
../Libraries/fs/ospi_nor_mx25lmxxx45g.c 

OBJS += \
./Libraries/fs/lfs_port_internal_nor_stm32h5.o \
./Libraries/fs/lfs_port_internal_nor_stm32u5.o \
./Libraries/fs/lfs_port_ospi.o \
./Libraries/fs/lfs_port_prv.o \
./Libraries/fs/lfs_port_xspi.o \
./Libraries/fs/ospi_nor_mx25lmxxx45g.o 

C_DEPS += \
./Libraries/fs/lfs_port_internal_nor_stm32h5.d \
./Libraries/fs/lfs_port_internal_nor_stm32u5.d \
./Libraries/fs/lfs_port_ospi.d \
./Libraries/fs/lfs_port_prv.d \
./Libraries/fs/lfs_port_xspi.d \
./Libraries/fs/ospi_nor_mx25lmxxx45g.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/fs/%.o Libraries/fs/%.su Libraries/fs/%.cyclo: ../Libraries/fs/%.c Libraries/fs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Libraries/fs/lfs_port_internal_nor_stm32h5.c_includes.args"

clean: clean-Libraries-2f-fs

clean-Libraries-2f-fs:
	-$(RM) ./Libraries/fs/lfs_port_internal_nor_stm32h5.cyclo ./Libraries/fs/lfs_port_internal_nor_stm32h5.d ./Libraries/fs/lfs_port_internal_nor_stm32h5.o ./Libraries/fs/lfs_port_internal_nor_stm32h5.su ./Libraries/fs/lfs_port_internal_nor_stm32u5.cyclo ./Libraries/fs/lfs_port_internal_nor_stm32u5.d ./Libraries/fs/lfs_port_internal_nor_stm32u5.o ./Libraries/fs/lfs_port_internal_nor_stm32u5.su ./Libraries/fs/lfs_port_ospi.cyclo ./Libraries/fs/lfs_port_ospi.d ./Libraries/fs/lfs_port_ospi.o ./Libraries/fs/lfs_port_ospi.su ./Libraries/fs/lfs_port_prv.cyclo ./Libraries/fs/lfs_port_prv.d ./Libraries/fs/lfs_port_prv.o ./Libraries/fs/lfs_port_prv.su ./Libraries/fs/lfs_port_xspi.cyclo ./Libraries/fs/lfs_port_xspi.d ./Libraries/fs/lfs_port_xspi.o ./Libraries/fs/lfs_port_xspi.su ./Libraries/fs/ospi_nor_mx25lmxxx45g.cyclo ./Libraries/fs/ospi_nor_mx25lmxxx45g.d ./Libraries/fs/ospi_nor_mx25lmxxx45g.o ./Libraries/fs/ospi_nor_mx25lmxxx45g.su

.PHONY: clean-Libraries-2f-fs

