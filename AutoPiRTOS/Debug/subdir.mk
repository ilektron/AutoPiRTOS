################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cr_startup_lpc13.c 

OBJS += \
./cr_startup_lpc13.o 

C_DEPS += \
./cr_startup_lpc13.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC13xx -D__CODE_RED -D__REDLIB__ -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/CMSISv1p30_LPC13xx/inc" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/FreeRTOS_Library/portable" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/FreeRTOS_Library/include" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS/inc" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS/src" -O2 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


