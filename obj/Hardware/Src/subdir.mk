################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hardware/Src/MPU6050.c \
../Hardware/Src/PWM.c \
../Hardware/Src/Varibles.c 

C_DEPS += \
./Hardware/Src/MPU6050.d \
./Hardware/Src/PWM.d \
./Hardware/Src/Varibles.d 

OBJS += \
./Hardware/Src/MPU6050.o \
./Hardware/Src/PWM.o \
./Hardware/Src/Varibles.o 

DIR_OBJS += \
./Hardware/Src/*.o \

DIR_DEPS += \
./Hardware/Src/*.d \

DIR_EXPANDS += \
./Hardware/Src/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
Hardware/Src/%.o: ../Hardware/Src/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"d:/MounRiver Project/Project/Debug" -I"d:/MounRiver Project/Project/Core" -I"d:/MounRiver Project/Project/User" -I"d:/MounRiver Project/Project/Peripheral/inc" -I"D:\MounRiver Project\Project\Hardware\Inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

