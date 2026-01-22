################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/%.o Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/%.su Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/%.cyclo: ../Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/%.c Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1 '-DMBEDTLS_CONFIG_FILE="mbedtls_config_ntz.h"' '-DLFS_CONFIG=lfs_config.h' -DLFS_USE_INTERNAL_NOR -DST67W6X_RCP -DST67_ARCH=W6X_ARCH_T02 -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.c_includes.args"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-Source

clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-Source:
	-$(RM) ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/croutine.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/event_groups.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/list.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/queue.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/stream_buffer.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/tasks.su ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.cyclo ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.d ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.o ./Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/timers.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_RTOS_FreeRTOS-2f-Source

