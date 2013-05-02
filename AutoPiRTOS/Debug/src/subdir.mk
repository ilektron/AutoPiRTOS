################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/gpio.c \
../src/hmc5883.c \
../src/i2c.c \
../src/i2ctask.c \
../src/integration.c \
../src/l3g4200d.c \
../src/main.c \
../src/pid.c \
../src/quaternion.c \
../src/timer32.c 

OBJS += \
./src/gpio.o \
./src/hmc5883.o \
./src/i2c.o \
./src/i2ctask.o \
./src/integration.o \
./src/l3g4200d.o \
./src/main.o \
./src/pid.o \
./src/quaternion.o \
./src/timer32.o 

C_DEPS += \
./src/gpio.d \
./src/hmc5883.d \
./src/i2c.d \
./src/i2ctask.d \
./src/integration.d \
./src/l3g4200d.d \
./src/main.d \
./src/pid.d \
./src/quaternion.d \
./src/timer32.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC13xx -D__CODE_RED -D__REDLIB__ -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/CMSISv1p30_LPC13xx/inc" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/FreeRTOS_Library/portable" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/FreeRTOS_Library/include" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS/inc" -I"/home/stephen/Dropbox/AutoPiRTOS_workspace/AutoPiRTOS/src" -O2 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


