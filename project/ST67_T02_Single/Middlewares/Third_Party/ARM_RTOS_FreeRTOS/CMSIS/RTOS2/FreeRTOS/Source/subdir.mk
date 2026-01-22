################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.c 

OBJS += \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/%.o Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/%.su Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/%.cyclo: ../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/%.c Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-CMSIS-2f-RTOS2-2f-FreeRTOS-2f-Source

clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-CMSIS-2f-RTOS2-2f-FreeRTOS-2f-Source:
	-$(RM) ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/freertos_evr.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-CMSIS-2f-RTOS2-2f-FreeRTOS-2f-Source

