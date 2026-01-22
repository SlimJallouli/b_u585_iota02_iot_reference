################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.c 

OBJS += \
./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.o 

C_DEPS += \
./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/%.o Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/%.su Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/%.cyclo: ../Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/%.c Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.c_includes.args"

clean: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Utils-2f-Misc

clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Utils-2f-Misc:
	-$(RM) ./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.cyclo ./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.d ./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.o ./Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/common_parser.su

.PHONY: clean-Middlewares-2f-ST-2f-ST67W6X_Network_Driver-2f-Utils-2f-Misc

