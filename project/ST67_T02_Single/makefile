################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ST67W6X_Network_Driver/Target/subdir.mk
-include Middlewares/Third_Party/lwIP_Network_lwIP/ports/freertos/subdir.mk
-include Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/netif/subdir.mk
-include Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/core/ipv4/subdir.mk
-include Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/core/subdir.mk
-include Middlewares/Third_Party/lwIP_Network_lwIP/lwip/src/api/subdir.mk
-include Middlewares/Third_Party/AWS_FreeRTOS/coreMQTT/source/subdir.mk
-include Middlewares/Third_Party/AWS_FreeRTOS/coreMQTT-Agent/source/subdir.mk
-include Middlewares/Third_Party/AWS_FreeRTOS/coreJSON/source/subdir.mk
-include Middlewares/Third_Party/AWS_FreeRTOS/backoffAlgorithm/source/subdir.mk
-include Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/portable/os/subdir.mk
-include Middlewares/Third_Party/AWS_AWS\ IoT/ota-for-aws-iot-embedded-sdk/source/subdir.mk
-include Middlewares/Third_Party/AWS_AWS\ IoT/Fleet-Provisioning-for-AWS-IoT-embedded-sdk/source/subdir.mk
-include Middlewares/Third_Party/AWS_AWS\ IoT/Device-Shadow-for-AWS-IoT-embedded-sdk/source/subdir.mk
-include Middlewares/Third_Party/AWS_AWS\ IoT/Device-Defender-for-AWS-IoT-embedded-sdk/source/subdir.mk
-include Middlewares/Third_Party/ARM_Security/library/subdir.mk
-include Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/subdir.mk
-include Middlewares/Third_Party/ARM_RTOS_FreeRTOS/Source/subdir.mk
-include Middlewares/Third_Party/ARM_RTOS_FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/subdir.mk
-include Middlewares/ST/ST67W6X_Network_Driver/Utils/Misc/subdir.mk
-include Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_bus/subdir.mk
-include Middlewares/ST/ST67W6X_Network_Driver/Driver/W61_at/subdir.mk
-include Middlewares/ST/ST67W6X_Network_Driver/Core/subdir.mk
-include Libraries/tinycbor/src/subdir.mk
-include Libraries/littlefs/subdir.mk
-include Libraries/fs/subdir.mk
-include Libraries/corePKCS11/source/dependency/3rdparty/mbedtls_utils/subdir.mk
-include Libraries/corePKCS11/source/subdir.mk
-include Drivers/STM32U5xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/SAFE_Axx0/subdir.mk
-include Drivers/BSP/Components/veml3235/subdir.mk
-include Drivers/BSP/Components/lps22hh/subdir.mk
-include Drivers/BSP/Components/ism330dhcx/subdir.mk
-include Drivers/BSP/Components/iis2mdc/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/stsafe/subdir.mk
-include Core/Src/ota_pal/subdir.mk
-include Core/Src/crypto/subdir.mk
-include Core/Src/corePKCS11/subdir.mk
-include Core/Src/FreeRTOS/subdir.mk
-include Core/Src/subdir.mk
-include Common/sys/subdir.mk
-include Common/net/lwip_port/subdir.mk
-include Common/net/W6X_ARCH_T02/subdir.mk
-include Common/kvstore/subdir.mk
-include Common/crypto/subdir.mk
-include Common/cli/subdir.mk
-include Common/app/sensors/subdir.mk
-include Common/app/pub_subscribe/subdir.mk
-include Common/app/ping/subdir.mk
-include Common/app/mqtt/subdir.mk
-include Common/app/led/subdir.mk
-include Common/app/echo_server/subdir.mk
-include Common/app/button/subdir.mk
-include Common/app/aws/shadow/subdir.mk
-include Common/app/aws/ota/subdir.mk
-include Common/app/aws/defender/subdir.mk
-include Common/app/aws/FleetProvisioning/subdir.mk
-include Common/app/HomeAssistant/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := b_u585_iota02_iot_reference
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
b_u585_iota02_iot_reference.elf \

MAP_FILES += \
b_u585_iota02_iot_reference.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
b_u585_iota02_iot_reference.list \

OBJCOPY_HEX += \
b_u585_iota02_iot_reference.hex \

OBJCOPY_BIN += \
b_u585_iota02_iot_reference.bin \


# All Target
all: main-build

# Main-build Target
main-build: b_u585_iota02_iot_reference.elf secondary-outputs

# Tool invocations
b_u585_iota02_iot_reference.elf b_u585_iota02_iot_reference.map: $(OBJS) $(USER_OBJS) C:\Users\stred\OneDrive\Documents\Projects\AWS\ST67\b_u585_iota02_iot_reference\project\STM32U585AIIXQ_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "b_u585_iota02_iot_reference.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"C:\Users\stred\OneDrive\Documents\Projects\AWS\ST67\b_u585_iota02_iot_reference\project\STM32U585AIIXQ_FLASH.ld" --specs=nosys.specs -Wl,-Map="b_u585_iota02_iot_reference.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

b_u585_iota02_iot_reference.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "b_u585_iota02_iot_reference.list"
	@echo 'Finished building: $@'
	@echo ' '

b_u585_iota02_iot_reference.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "b_u585_iota02_iot_reference.hex"
	@echo 'Finished building: $@'
	@echo ' '

b_u585_iota02_iot_reference.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "b_u585_iota02_iot_reference.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) b_u585_iota02_iot_reference.bin b_u585_iota02_iot_reference.elf b_u585_iota02_iot_reference.hex b_u585_iota02_iot_reference.list b_u585_iota02_iot_reference.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
