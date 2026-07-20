################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v30x_it.c \
../User/main.c \
../User/system_ch32v30x.c 

C_DEPS += \
./User/ch32v30x_it.d \
./User/main.d \
./User/system_ch32v30x.d 

OBJS += \
./User/ch32v30x_it.o \
./User/main.o \
./User/system_ch32v30x.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/15256/Desktop/QIANSAI/Debug" -I"c:/Users/15256/Desktop/QIANSAI/Core" -I"c:/Users/15256/Desktop/QIANSAI/User" -I"c:/Users/15256/Desktop/QIANSAI/Peripheral/inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Src" -isystem"c:/Users/15256/Desktop/QIANSAI/Hardware" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

