################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Voltageandcurrentlogger.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Voltageandcurrentlogger.list \

OBJCOPY_BIN += \
Voltageandcurrentlogger.bin \


# All Target
all: Voltageandcurrentlogger.elf secondary-outputs

# Tool invocations
Voltageandcurrentlogger.elf: $(OBJS) $(USER_OBJS) D:\[0]_my_all_project\1_Voltage\ and\ Current\ Logger\Voltage\ and\ Current\ Logger\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Voltageandcurrentlogger.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\[0]_my_all_project\1_Voltage and Current Logger\Voltage and Current Logger\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Voltageandcurrentlogger.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Voltageandcurrentlogger.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Voltageandcurrentlogger.list"
	@echo 'Finished building: $@'
	@echo ' '

Voltageandcurrentlogger.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Voltageandcurrentlogger.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
