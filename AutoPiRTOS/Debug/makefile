################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: AutoPiRTOS.axf

# Tool invocations
AutoPiRTOS.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/stephen/Dropbox/AutoPiRTOS_workspace/FreeRTOS_Library/Debug" -L"/home/stephen/Dropbox/AutoPiRTOS_workspace/CMSISv1p30_LPC13xx/Debug" -Xlinker -Map=AutoPiRTOS.map -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "AutoPiRTOS_Debug.ld" -o "AutoPiRTOS.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) AutoPiRTOS.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size AutoPiRTOS.axf; # arm-none-eabi-objcopy -O ihex AutoPiRTOS.axf AutoPiRTOS.hex ;
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
