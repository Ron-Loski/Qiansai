################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hardware/Src/ArmKinematics.c \
../Hardware/Src/ArmServo.c \
../Hardware/Src/Astar.c \
../Hardware/Src/CallBack.c \
../Hardware/Src/Encoder.c \
../Hardware/Src/MPU6050.c \
../Hardware/Src/Motor.c \
../Hardware/Src/PID.c \
../Hardware/Src/Timer.c \
../Hardware/Src/Varibles.c \
../Hardware/Src/usart.c 

C_DEPS += \
./Hardware/Src/ArmKinematics.d \
./Hardware/Src/ArmServo.d \
./Hardware/Src/Astar.d \
./Hardware/Src/CallBack.d \
./Hardware/Src/Encoder.d \
./Hardware/Src/MPU6050.d \
./Hardware/Src/Motor.d \
./Hardware/Src/PID.d \
./Hardware/Src/Timer.d \
./Hardware/Src/Varibles.d \
./Hardware/Src/usart.d 

OBJS += \
./Hardware/Src/ArmKinematics.o \
./Hardware/Src/ArmServo.o \
./Hardware/Src/Astar.o \
./Hardware/Src/CallBack.o \
./Hardware/Src/Encoder.o \
./Hardware/Src/MPU6050.o \
./Hardware/Src/Motor.o \
./Hardware/Src/PID.o \
./Hardware/Src/Timer.o \
./Hardware/Src/Varibles.o \
./Hardware/Src/usart.o 

DIR_OBJS += \
./Hardware/Src/*.o \

DIR_DEPS += \
./Hardware/Src/*.d \

DIR_EXPANDS += \
./Hardware/Src/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Hardware/Src/%.o: ../Hardware/Src/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/15256/Desktop/QIANSAI/Debug" -I"c:/Users/15256/Desktop/QIANSAI/Core" -I"c:/Users/15256/Desktop/QIANSAI/User" -I"c:/Users/15256/Desktop/QIANSAI/Peripheral/inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Src" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"


